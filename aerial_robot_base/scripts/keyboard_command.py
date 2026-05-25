#!/usr/bin/env python
import math
import rospy

from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
import rosgraph

import sys, select, termios, tty

current_odom = None


def odomCallback(msg):
        global current_odom
        current_odom = msg


def normalizeAngle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def yawFromOdom(odom):
        q = odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def hasSubscriber(topic_name):
        master = rosgraph.Master('/keyboard_command')
        try:
                _, subs, _ = master.getSystemState()
        except Exception:
                return False

        return any(topic == topic_name and len(nodes) > 0 for topic, nodes in subs)


def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        robot_ns = rospy.get_param("~robot_ns", "");
        pos_step = rospy.get_param("~pos_step", 0.1)
        z_step = rospy.get_param("~z_step", 0.05)
        yaw_step = rospy.get_param("~yaw_step", 0.1)

        if not robot_ns:
                master = rosgraph.Master('/rostopic')
                try:
                        _, subs, _ = master.getSystemState()

                except socket.error:
                        raise ROSTopicIOException("Unable to communicate with master!")

                teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
                if len(teleop_topics) == 1:
                        robot_ns = teleop_topics[0].split('/teleop')[0]

        ns = robot_ns + "/teleop_command"
        land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
        halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        ctrl_mode_pub = rospy.Publisher(ns + '/ctrl_mode', Int8, queue_size=1)
        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)
        nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=1)
        agg_land_pub = rospy.Publisher(robot_ns + '/agg_state/land', Empty, queue_size=1)
        odom_sub = rospy.Subscriber(robot_ns + '/uav/cog/odom', Odometry, odomCallback, queue_size=1)


        #the way to write publisher in python
        comm=Int8()
        gain=UInt16()

        def publishPositionCommand(dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
                if current_odom is None:
                        rospy.logwarn("No odometry received yet. Cannot publish keyboard position command.")
                        return

                pos = current_odom.pose.pose.position
                yaw = yawFromOdom(current_odom)

                nav_msg = FlightNav()
                nav_msg.header.stamp = rospy.Time.now()
                nav_msg.control_frame = FlightNav.WORLD_FRAME
                nav_msg.target = FlightNav.COG
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                nav_msg.target_pos_x = pos.x + dx
                nav_msg.target_pos_y = pos.y + dy
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                nav_msg.target_yaw = normalizeAngle(yaw + dyaw)
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                nav_msg.target_pos_z = pos.z + dz
                nav_pub.publish(nav_msg)

        try:
                while(True):
                        key = getKey()
                        print("the key value is {}".format(ord(key)))
                        # takeoff and landing
                        if key == 'l':
                                agg_land_topic = robot_ns + '/agg_state/land'
                                if hasSubscriber(agg_land_topic):
                                        agg_land_pub.publish(Empty())
                                        rospy.logwarn("agg_state is active. Requested soft landing through %s.", agg_land_topic)
                                else:
                                        land_pub.publish(Empty())
                                #for hydra joints
                        if key == 'r':
                                start_pub.publish(Empty())
                                #for hydra joints
                        if key == 'h':
                                halt_pub.publish(Empty())
                                 #for hydra joints
                        if key == 'f':
                                force_landing_pub.publish(Empty())
                        if key == 't':
                                takeoff_pub.publish(Empty())
                        if key == 'u':
                                rospy.logwarn("stair command publisher is not configured.")
                        if key == 'x':
                                motion_start_pub.publish()
                        if key == 'v':
                                comm.data = 1
                                ctrl_mode_pub.publish(comm)
                        if key == 'p':
                                comm.data = 0
                                ctrl_mode_pub.publish(comm)
                        # position/yaw command, matching joystick-style small position steps
                        if key == 'w':
                                publishPositionCommand(dx=pos_step)
                        if key == 's':
                                publishPositionCommand(dx=-pos_step)
                        if key == 'a':
                                publishPositionCommand(dy=pos_step)
                        if key == 'd':
                                publishPositionCommand(dy=-pos_step)
                        if key == 'z':
                                publishPositionCommand(dz=z_step)
                        if key == 'c':
                                publishPositionCommand(dz=-z_step)
                        if key == 'q':
                                publishPositionCommand(dyaw=yaw_step)
                        if key == 'e':
                                publishPositionCommand(dyaw=-yaw_step)
                        if key == '\x03':
                                break
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
