#!/usr/bin/env python

import rospy
import math

from sami.arm import Arm
from sami.gripper import Gripper
from iris_sami.srv import Status, Velocity, JointGoal, JointGoalName, SaveJointGoalName, Actionlist, LoadJointGoalName, PoseGoal, RelativeMove, NoArguments

arm = None
gripper = None


def info_srv(req):
    rospy.loginfo('Status service called')
    feedback = 'Robot is operational'
    joints = arm.get_joints()
    pose = arm.get_pose()
    velocity = arm.velocity
    return [True, feedback, joints, pose, velocity]


def velocity_srv(req):
    rospy.loginfo('Velocity service called with value ' + str(req.velocity))
    arm.velocity = req.velocity
    if arm.velocity != req.velocity:
        return [False, 'An error has occured']
    return [True, 'Velocity set to ' + str(req.velocity)]


def move_joint_srv(req):
    rospy.loginfo('Joint Goal service called with values\n' + str(req))
    joints = [req.shoulder_pan, req.shoulder_lift, req.elbow, req.wrist_1, req.wrist_2, req.wrist_3]
    ok = arm.move_joints(joints)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Success']

def move_joint_name_srv(req):
    pos_name = req.joint_position_name

    ok = arm.move_joints_alias(pos_name)

    if not ok:
        return [False, arm.error_msg]
    return [True, 'Success']

def save_joint_name_srv(req):
    position = req.position_name
    ok = arm.save_joint_position_alias(position)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Success']

def load_joint_name_srv(req):
    
    filename = req.joint_positions_file
    ok = arm.load_joint_position_aliases(filename)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Success']

def actionlist_srv(req):

    filename = req.actionlist_filename
    ok = arm.execute_actionlist(filename)
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


def grip_srv(req):
    rospy.loginfo('Grip service called')
    result = gripper.grip()
    if result:
        return [False, 'An error has ocurred']
    return [True, 'Gripper is Closed']


def release_srv(req):
    rospy.loginfo('Release service called')
    result = gripper.release()
    if result:
        return [False, 'An error has ocurred']
    return [True, 'Gripper is Opened']


def main():
    rospy.init_node('sami_server', anonymous=True)

    rospy.Service('/iris_sami/status', Status, info_srv)
    rospy.Service('/iris_sami/velocity', Velocity, velocity_srv)
    rospy.Service('/iris_sami/joints', JointGoal, move_joint_srv)
    rospy.Service('/iris_sami/joints_alias', JointGoalName, move_joint_name_srv)
    rospy.Service('/iris_sami/save_joints_alias', SaveJointGoalName, save_joint_name_srv)
    rospy.Service('/iris_sami/load_joints_alias', LoadJointGoalName, load_joint_name_srv)
    rospy.Service('/iris_sami/actionlist', Actionlist, actionlist_srv)
    rospy.Service('/iris_sami/pose', PoseGoal, move_pose_srv)
    rospy.Service('/iris_sami/move', RelativeMove, move_pose_relative_srv)
    rospy.Service('/iris_sami/grip', NoArguments, grip_srv)
    rospy.Service('/iris_sami/release', NoArguments, release_srv)

    global arm, gripper
    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 0.2

    gripper = Gripper('cr200-85', host='localhost', port=44221)

    rospy.spin()


if __name__ == '__main__':
    main()