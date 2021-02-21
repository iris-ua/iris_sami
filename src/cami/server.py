#!/usr/bin/env python

import rospy
import math

from sami.arm import Arm
from sami.gripper import Gripper
from iris_sami.srv import JointGoal, PoseGoal, RelativeMove

arm = None

def move_joint_srv(req):
    joints = [req.shoulder_pan, req.shoulder_lift, req.elbow, req.wrist_1, req.wrist_2, req.wrist_3]
    ok = arm.move_joints(joints)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Success']


def move_pose_srv(req):
    pose = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    ok = arm.move_pose(pose)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Success']


def move_pose_relative_srv(req):
    pose = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    ok = arm.move_pose_relative(pose)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Success']


def main():
    rospy.init_node('sami_server', anonymous=True)

    rospy.Service('/iris_sami/joints', JointGoal, move_joint_srv)
    rospy.Service('/iris_sami/pose', PoseGoal, move_pose_srv)
    rospy.Service('/iris_sami/move', RelativeMove, move_pose_relative_srv)

    print('service setup')

    global arm
    arm = Arm('ur10e', group='manipulator')
    arm.velocity = 0.2

    rospy.spin()


if __name__ == '__main__':
    main()