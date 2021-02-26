
import sys
import copy
import rospy
import socket

import urx

import numpy as np
import tf.transformations as tr
from moveit_commander.conversions import list_to_pose, pose_to_list

from sami.interface import ArmIF

'''
    Reference API for URScript
    https://s3-eu-west-1.amazonaws.com/ur-support-site/18679/scriptmanual_en.pdf

    Examples:
    https://s3-eu-west-1.amazonaws.com/ur-support-site/29983/Script%20command%20Examples.pdf


    Two different approaches to control the robot using URScript:
        1. Using the drivers already existing topic for that specific purpose ('ur_hardware_interface/script_command')
           (internally uses this stream -> std::unique_ptr<comm::URStream<ur_driver::primary_interface::PackageHeader>> secondary_stream_;
           to ultimatelly write to the robot)
        2. Directly open a TCP connection with the robot (robot_ip:3002) and send the correctly formatted strings 


'''

class URScriptPlug(ArmIF):
    def __init__(self, options):
        super(URScriptPlug, self).__init__()
        self.rob = urx.Robot("10.1.0.2")
        print(self.rob.getj())
        print(self.rob.is_running())

    def move_joints(self, joints, velocity):
        if velocity is None:
            velocity = self.velocity

        try:
            self.rob.movej(tuple(joints), vel=velocity) 
        except Exception as e:
            self.last_error_msg = str(e)
            return False
        return True

    def move_pose(self, pose, velocity):
        if velocity is None:
            velocity = self.velocity
        
        try:
            self.rob.movel(tuple(pose), vel=velocity)
            return True
        except Exception as e:
            self.last_error_msg = str(e)
            return False

    def move_pose_relative(self, dpose, velocity):
        if velocity is None:
            velocity = self.velocity

        try:
            self.rob.movel(tuple(dpose), vel=velocity, relative=True)
        except Exception as e:
            self.last_error_msg = str(e)
            return False
        return True
    
    # def move_pose_relative_world(self, dpose, velocity):
    #     if velocity is None:
    #         self.moveg.set_max_velocity_scaling_factor(self.velocity)
    #     else:
    #         self.moveg.set_max_velocity_scaling_factor(velocity)

    #     pose = pose_to_list(self.moveg.get_current_pose().pose)
    #     pose = np.sum(pose, dpose)

    #     wp = [list_to_pose(pose)]
    #     (plan, fraction) = self.moveg.compute_cartesian_path(wp, eef_step = 0.01, jump_threshold = 0.0)
    #     if fraction < 1.0:
    #         self.last_error_msg = "No motion plan found."
    #         return False

    #     v = self.velocity if velocity is None else velocity
    #     plan = self.moveg.retime_trajectory(self.robot.get_current_state(), plan, v)

    #     try:
    #         self.moveg.execute(plan, wait=True)
    #         self.moveg.stop()
    #     except MoveItCommanderException as e:
    #         self.last_error_msg = str(e)
    #         return False

    #     return True

    
    def get_joints(self):
        print ("Current joints is: ",  self.rob.getj())
        return self.rob.getj()



    def get_pose(self):
        print ("Current tool pose is: ",  self.rob.getl())
        return list_to_pose(self.rob.getl())
