#!/usr/bin/env python3

import argparse
import sys

import amb_trans
import rospy
from aerial_robot_model.srv import AddExtraModule
from geometry_msgs.msg import Inertia, Quaternion, Transform, Vector3


ROBOT_NS = "beetle1"
MODULE_NAME = "amb_box"
PARENT_LINK = "main_body"

# This is effectively relative to the amoeba root frame. The URDF root link is
# a dummy link, while the model's inertial root is main_body with an identity
# fixed joint from root.
MODULE_TRANSLATION = (0.0, 0.0, -0.1)
MODULE_ROTATION = (0.0, 0.0, 0.0, 1.0)

# Outer size is 0.4 x 0.4 x 0.4 m, mass is 1 kg. Approximate inertia with the
# mass uniformly concentrated in the lower 0.4 x 0.4 x 0.1 m cuboid.
MASS = 1.0
MASS_DISTRIBUTION_SIZE = (0.4, 0.4, 0.1)


def normalize_ns(robot_ns):
    return "/" + robot_ns.strip("/")


def parse_length(argv):
    for arg in argv:
        if arg.startswith("len="):
            return arg.split("=", 1)[1]
    return None


def move_gripper_to_length(length_arg, robot_ns):
    length = amb_trans.parse_joint_argument(length_arg)
    if length is None:
        return 1
    if length < 0 or length > 100:
        rospy.logerr("Length %s out of range [0, 100]", length)
        return 1

    rospy.set_param("~robot_ns", robot_ns)
    node = amb_trans.ServoMoveNode(init_node=False)
    node.move_to_position(amb_trans.length2angle(length))
    rospy.loginfo("Moved gripper to len=%s", length)
    return 0


def make_transform():
    transform = Transform()
    transform.translation = Vector3(*MODULE_TRANSLATION)
    transform.rotation = Quaternion(*MODULE_ROTATION)
    return transform


def make_inertia():
    lx, ly, lz = MASS_DISTRIBUTION_SIZE

    inertia = Inertia()
    inertia.m = MASS
    inertia.com = Vector3(0.0, 0.0, -0.15)
    inertia.ixx = MASS * (ly * ly + lz * lz) / 12.0
    inertia.iyy = MASS * (lx * lx + lz * lz) / 12.0
    inertia.izz = MASS * (lx * lx + ly * ly) / 12.0
    inertia.ixy = 0.0
    inertia.ixz = 0.0
    inertia.iyz = 0.0
    return inertia


def main():
    parser = argparse.ArgumentParser(description="Add the fixed amoeba box payload to the robot model.")
    parser.add_argument("--robot-ns", default=ROBOT_NS, help="Robot namespace, e.g. beetle1.")
    parser.add_argument("-r", "--remove", action="store_true", help="Remove the box payload.")
    parser.add_argument("--timeout", type=float, default=5.0, help="Service wait timeout [s].")
    ros_args = rospy.myargv(argv=sys.argv)[1:]
    args, unknown_args = parser.parse_known_args(ros_args)
    length_arg = parse_length(unknown_args)

    rospy.init_node("amb_box")
    if length_arg is not None:
        move_status = move_gripper_to_length(length_arg, args.robot_ns)
        if move_status != 0:
            return move_status

    service_name = normalize_ns(args.robot_ns) + "/add_extra_module"

    try:
        rospy.wait_for_service(service_name, timeout=args.timeout)
    except rospy.ROSException as exc:
        rospy.logerr("Timed out waiting for %s: %s", service_name, exc)
        return 1

    req = AddExtraModule._request_class()
    req.module_name = MODULE_NAME

    if args.remove:
        req.action = AddExtraModule._request_class.REMOVE
    else:
        req.action = AddExtraModule._request_class.ADD
        req.parent_link_name = PARENT_LINK
        req.transform = make_transform()
        req.inertia = make_inertia()

    response = rospy.ServiceProxy(service_name, AddExtraModule)(req)
    if response.status:
        action = "removed" if args.remove else "added"
        rospy.loginfo("%s %s through %s", action, MODULE_NAME, service_name)
        return 0

    rospy.logerr("Failed to update %s through %s", MODULE_NAME, service_name)
    return 2


if __name__ == "__main__":
    sys.exit(main())
