#!/usr/bin/env python3
# rosrun aerial_robot_planning agg_state.py robot_name=beetle1 roll=90 pitch=0 yaw=nan
import math
import re
import sys
import threading

import rospy
import tf_conversions as tf
from geometry_msgs.msg import Quaternion, Transform, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


PS3_AXES = 29
PS3_AXIS_STICK_LEFT_LEFTWARDS = 0
PS3_AXIS_STICK_LEFT_UPWARDS = 1
PS3_AXIS_STICK_RIGHT_LEFTWARDS = 2
PS3_AXIS_STICK_RIGHT_UPWARDS = 3
PS3_BUTTON_REAR_LEFT_2 = 8

PS4_AXES = 8
PS4_AXIS_STICK_LEFT_LEFTWARDS = 0
PS4_AXIS_STICK_LEFT_UPWARDS = 1
PS4_AXIS_STICK_RIGHT_LEFTWARDS = 2
PS4_AXIS_STICK_RIGHT_UPWARDS = 5
PS4_BUTTON_REAR_LEFT_2 = 6


def parse_cli_key_values(argv):
    values = {}
    for arg in argv:
        match = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)=(-?\d+(?:\.\d+)?|nan|[-A-Za-z0-9_/]+)$", arg)
        if match:
            values[match.group(1)] = match.group(2)
    return values


def shortest_angular_distance(src, dst):
    return math.atan2(math.sin(dst - src), math.cos(dst - src))


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class AggStateNode:
    def __init__(self):
        self.cli_args = parse_cli_key_values(sys.argv[1:])

        self.robot_name = rospy.get_param("~robot_name", self.cli_args.get("robot_name", "beetle1"))
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "cog")
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.angle_unit = rospy.get_param("~angle_unit", "deg")

        self.max_xy_vel = rospy.get_param("~max_xy_vel", 0.3)
        self.max_z_vel = rospy.get_param("~max_z_vel", 0.15)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 0.4)
        self.max_angle_rate = rospy.get_param("~max_angle_rate", 0.35)
        self.deadzone = rospy.get_param("~deadzone", 0.08)
        self.local_xy_with_l2 = rospy.get_param("~local_xy_with_l2", True)
        self.enable_joy_yaw = rospy.get_param("~enable_joy_yaw", False)
        self.flatten_land_tol = rospy.get_param("~flatten_land_tol", 0.08)
        self.flatten_land_hold = rospy.get_param("~flatten_land_hold", 0.5)

        self.t_step = rospy.get_param(f"/{self.robot_name}/controller/nmpc/T_step", 0.1)
        self.n_nmpc = rospy.get_param(f"/{self.robot_name}/controller/nmpc/NN", 20)

        self.target_pos = None
        self.cmd_rpy = None
        self.target_rpy = None
        self.last_odom = None
        self.last_joy = None
        self.last_time = None
        self.active = True
        self.landing_requested = False
        self.flatten_reached_since = None
        self.lock = threading.Lock()

        self.roll_param = rospy.get_param("~roll", self.cli_args.get("roll", "nan"))
        self.pitch_param = rospy.get_param("~pitch", self.cli_args.get("pitch", "nan"))
        self.yaw_param = rospy.get_param("~yaw", self.cli_args.get("yaw", "nan"))

        self.traj_pub = rospy.Publisher(f"/{self.robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=3)
        self.land_pub = rospy.Publisher(f"/{self.robot_name}/teleop_command/land", Empty, queue_size=1)
        self.odom_sub = rospy.Subscriber(f"/{self.robot_name}/uav/cog/odom", Odometry, self.odom_cb, queue_size=1)
        self.joy_sub = rospy.Subscriber(f"/{self.robot_name}/joy", Joy, self.joy_cb, queue_size=1)

        self.input_thread = threading.Thread(target=self.stdin_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def odom_cb(self, msg):
        with self.lock:
            self.last_odom = msg
            if self.target_pos is not None:
                return

            pos = msg.pose.pose.position
            quat = msg.pose.pose.orientation
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.target_pos = [pos.x, pos.y, pos.z]

            current_rpy = [roll, pitch, yaw]
            target_rpy = [
                self.param_angle_or_current(self.roll_param, roll),
                self.param_angle_or_current(self.pitch_param, pitch),
                self.param_angle_or_current(self.yaw_param, yaw),
            ]
            self.cmd_rpy = current_rpy[:]
            self.target_rpy = target_rpy[:]

            rospy.loginfo(
                "agg_state initialized at pos [%.3f, %.3f, %.3f], current rpy [%.1f, %.1f, %.1f] deg, target rpy [%.1f, %.1f, %.1f] deg",
                self.target_pos[0],
                self.target_pos[1],
                self.target_pos[2],
                math.degrees(self.cmd_rpy[0]),
                math.degrees(self.cmd_rpy[1]),
                math.degrees(self.cmd_rpy[2]),
                math.degrees(target_rpy[0]),
                math.degrees(target_rpy[1]),
                math.degrees(target_rpy[2]),
            )

    def joy_cb(self, msg):
        with self.lock:
            self.last_joy = msg

    def param_angle_or_current(self, value, current):
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return current
        if math.isnan(parsed):
            return current
        return self.to_rad(parsed)

    def to_rad(self, value):
        if self.angle_unit == "rad":
            return value
        return math.radians(value)

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def get_joy_axes(self, joy):
        if joy is None:
            return 0.0, 0.0, 0.0, 0.0, False

        if len(joy.axes) == PS3_AXES:
            lx = joy.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]
            ly = joy.axes[PS3_AXIS_STICK_LEFT_UPWARDS]
            rx = joy.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS]
            ry = joy.axes[PS3_AXIS_STICK_RIGHT_UPWARDS]
            local = len(joy.buttons) > PS3_BUTTON_REAR_LEFT_2 and joy.buttons[PS3_BUTTON_REAR_LEFT_2] == 1
            return lx, ly, rx, ry, local

        if len(joy.axes) >= PS4_AXES:
            lx = joy.axes[PS4_AXIS_STICK_LEFT_LEFTWARDS]
            ly = joy.axes[PS4_AXIS_STICK_LEFT_UPWARDS]
            rx = joy.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS]
            ry = joy.axes[PS4_AXIS_STICK_RIGHT_UPWARDS]
            local = len(joy.buttons) > PS4_BUTTON_REAR_LEFT_2 and joy.buttons[PS4_BUTTON_REAR_LEFT_2] == 1
            return lx, ly, rx, ry, local

        rospy.logwarn_throttle(1.0, "Unsupported joystick layout: axes=%d buttons=%d", len(joy.axes), len(joy.buttons))
        return 0.0, 0.0, 0.0, 0.0, False

    def stdin_loop(self):
        help_text = (
            "agg_state commands: roll=90 pitch=0 yaw=0 | r=90 p=0 y=0 | l/land | q\n"
            "angles are degrees by default; set _angle_unit:=rad for radians."
        )
        rospy.loginfo(help_text)

        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                rospy.signal_shutdown("agg_state quit command")
                return
            if line in ("l", "land"):
                with self.lock:
                    if self.target_rpy is not None:
                        self.target_rpy[0] = 0.0
                        self.target_rpy[1] = 0.0
                    self.landing_requested = True
                    self.flatten_reached_since = None
                rospy.logwarn("Landing requested: flattening roll/pitch before publishing land command.")
                continue

            updates = self.parse_rpy_command(line)
            if not updates:
                rospy.logwarn("Unknown agg_state command: %s", line)
                continue

            with self.lock:
                if self.target_rpy is None:
                    rospy.logwarn("Odometry is not ready yet; command ignored.")
                    continue
                for idx, value in updates.items():
                    self.target_rpy[idx] = self.to_rad(value)
                self.landing_requested = False
                self.active = True
            rospy.loginfo("Updated target rpy command: %s", line)

    def parse_rpy_command(self, line):
        mapping = {"roll": 0, "r": 0, "pitch": 1, "p": 1, "yaw": 2, "y": 2}
        updates = {}
        for key, value in re.findall(r"(roll|pitch|yaw|r|p|y)\s*=\s*(-?\d+(?:\.\d+)?)", line):
            updates[mapping[key]] = float(value)
        return updates

    def step_towards_rpy(self, dt):
        max_step = self.max_angle_rate * dt
        for i in range(3):
            delta = shortest_angular_distance(self.cmd_rpy[i], self.target_rpy[i])
            self.cmd_rpy[i] += clamp(delta, -max_step, max_step)

    def update_target_from_joy(self, dt):
        lx, ly, rx, ry, local = self.get_joy_axes(self.last_joy)
        lx = self.apply_deadzone(lx)
        ly = self.apply_deadzone(ly)
        rx = self.apply_deadzone(rx)
        ry = self.apply_deadzone(ry)

        vx = ly * abs(ly) * self.max_xy_vel
        vy = lx * abs(lx) * self.max_xy_vel
        vz = ry * abs(ry) * self.max_z_vel

        if self.local_xy_with_l2 and local:
            yaw = self.cmd_rpy[2]
            world_vx = math.cos(yaw) * vx - math.sin(yaw) * vy
            world_vy = math.sin(yaw) * vx + math.cos(yaw) * vy
            vx, vy = world_vx, world_vy

        self.target_pos[0] += vx * dt
        self.target_pos[1] += vy * dt
        self.target_pos[2] += vz * dt

        if self.enable_joy_yaw and not self.landing_requested:
            self.target_rpy[2] += rx * abs(rx) * self.max_yaw_rate * dt

    def build_traj(self, now):
        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(
            self.cmd_rpy[0], self.cmd_rpy[1], self.cmd_rpy[2]
        )

        traj = MultiDOFJointTrajectory()
        traj.header.stamp = now
        traj.header.frame_id = self.frame_id
        traj.joint_names.append(self.child_frame_id)

        for i in range(self.n_nmpc + 1):
            pt = MultiDOFJointTrajectoryPoint()
            pt.transforms.append(
                Transform(
                    translation=Vector3(self.target_pos[0], self.target_pos[1], self.target_pos[2]),
                    rotation=Quaternion(qx, qy, qz, qw),
                )
            )
            pt.velocities.append(Twist())
            pt.accelerations.append(Twist())
            pt.time_from_start = rospy.Duration.from_sec(i * self.t_step)
            traj.points.append(pt)

        return traj

    def maybe_land(self, now):
        if not self.landing_requested:
            return

        flat_err = math.hypot(
            shortest_angular_distance(self.cmd_rpy[0], 0.0),
            shortest_angular_distance(self.cmd_rpy[1], 0.0),
        )

        if flat_err > self.flatten_land_tol:
            self.flatten_reached_since = None
            return

        if self.flatten_reached_since is None:
            self.flatten_reached_since = now.to_sec()
            return

        if now.to_sec() - self.flatten_reached_since < self.flatten_land_hold:
            return

        self.active = False
        self.landing_requested = False
        self.land_pub.publish(Empty())
        rospy.logwarn("agg_state flattened. Published /%s/teleop_command/land and stopped /set_ref_traj.", self.robot_name)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            with self.lock:
                if self.target_pos is not None and self.cmd_rpy is not None and self.active:
                    if self.last_time is None:
                        dt = 1.0 / self.rate_hz
                    else:
                        dt = max(0.0, min(0.1, (now - self.last_time).to_sec()))

                    self.update_target_from_joy(dt)
                    self.step_towards_rpy(dt)
                    self.traj_pub.publish(self.build_traj(now))
                    self.maybe_land(now)
                    self.last_time = now

            rate.sleep()


def main():
    rospy.init_node("agg_state")
    node = AggStateNode()
    node.spin()


if __name__ == "__main__":
    main()
