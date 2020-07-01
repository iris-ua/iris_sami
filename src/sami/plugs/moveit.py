
import sys
import copy

import moveit_commander

from moveit_commander.exception import MoveItCommanderException
from moveit_commander.conversions import list_to_pose, pose_to_list

import numpy as np
import tf.transformations as tr

from sami.interface import ArmIF

class MoveItPlug(ArmIF):
    def __init__(self, options):
        super(MoveItPlug, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.moveg = moveit_commander.MoveGroupCommander(options['group'])

        self.ee_link = self.moveg.get_end_effector_link()
        self.pframe = self.moveg.get_planning_frame()
        self.numj = len(self.moveg.get_joints())

    def move_joints(self, joints, velocity):
        if velocity is None:
            self.moveg.set_max_velocity_scaling_factor(self.velocity)
        else:
            self.moveg.set_max_velocity_scaling_factor(velocity)

        try:
            self.moveg.go(joints, wait=True)
            self.moveg.stop()
        except MoveItCommanderException as e:
            self.last_error_msg = str(e)
            return False
        return True

    def move_pose(self, pose, velocity):
        if velocity is None:
            self.moveg.set_max_velocity_scaling_factor(self.velocity)
        else:
            self.moveg.set_max_velocity_scaling_factor(velocity)

        try:
            self.moveg.set_pose_target(pose)
            ok = self.moveg.go(wait=True)
            self.moveg.stop()
            self.moveg.clear_pose_targets()
            if not ok:
                self.last_error_msg = "No motion plan found."
            return ok
        except MoveItCommanderException as e:
            self.last_error_msg = str(e)
            self.moveg.clear_pose_targets()
            return False

    def move_pose_relative(self, dpose, velocity):
        if velocity is None:
            self.moveg.set_max_velocity_scaling_factor(self.velocity)
        else:
            self.moveg.set_max_velocity_scaling_factor(velocity)

        pose = pose_to_list(self.moveg.get_current_pose().pose)

        t_xform = np.dot(tr.translation_matrix(pose[0:3]), tr.quaternion_matrix(pose[3:]))
        s_xform = np.dot(tr.translation_matrix(dpose[0:3]), tr.euler_matrix(*dpose[3:]))

        xform = np.dot(t_xform, s_xform)
        pose = tr.translation_from_matrix(xform).tolist() + list(tr.euler_from_matrix(xform))

        wp = [list_to_pose(pose)]  # waypoints
        (plan, fraction) = self.moveg.compute_cartesian_path(wp, eef_step = 0.01, jump_threshold = 0.0)
        if fraction < 1.0:
            self.last_error_msg = "No motion plan found."
            return False

        v = self.velocity if velocity is None else velocity
        plan = self.moveg.retime_trajectory(self.robot.get_current_state(), plan, v)

        try:
            self.moveg.execute(plan, wait=True)
            self.moveg.stop()
        except MoveItCommanderException as e:
            self.last_error_msg = str(e)
            return False

        return True
