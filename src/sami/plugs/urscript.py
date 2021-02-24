
import sys
import copy
import rospy
import socket

import numpy as np
import tf.transformations as tr

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
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.s.connect((options["host"], options["port"]))
        except Exception as e:
            print("Can't connect to the robot")
            print(e)

    def _sendCommand(self, command):
        self.s.send(command + "\n")
        response = self.s.recv(1024) # TODO: Check response values for errors etc.        
        return response

    def move_joints(self, joints, velocity):
        if velocity is None:
            velocity = self.velocity

        if velocity < 0 or velocity > 1: return False

        try: ## TODO: Make sure joints is float, python puts this as strings sometimes, the function demands numbers
            command = "movej(" + joints + ", v=" + velocity + ")"
            response = self._sendCommand(command)
            ## TODO: Check response? maybe it can fail?
        except Exception as e:
            self.last_error_msg = str(e)
            return False
        return True

    def move_pose(self, pose, velocity):
        if velocity is None:
            velocity = self.velocity

        if velocity < 0 or velocity > 1: return False

        try:
            command = "movep(" + pose + ", v=" + velocity + ")" # TODO: verificar comando movel com os mesmos parametros (move linear)
            response = self._sendCommand(command)
            ## TODO: Check respones? maybe it can fail
            # if not ok:
            #     self.last_error_msg = "No motion plan found."
            return True
        except Exception as e:
            self.last_error_msg = str(e)
            return False

    # def move_pose_relative(self, dpose, velocity):
    #     if velocity is None:
    #         self.moveg.set_max_velocity_scaling_factor(self.velocity)
    #     else:
    #         self.moveg.set_max_velocity_scaling_factor(velocity)

    #     pose = pose_to_list(self.moveg.get_current_pose().pose)

    #     t_xform = np.dot(tr.translation_matrix(pose[0:3]), tr.quaternion_matrix(pose[3:]))
    #     s_xform = np.dot(tr.translation_matrix(dpose[0:3]), tr.euler_matrix(*dpose[3:]))

    #     xform = np.dot(t_xform, s_xform)
    #     pose = tr.translation_from_matrix(xform).tolist() + list(tr.euler_from_matrix(xform))

    #     wp = [list_to_pose(pose)]  # waypoints
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
        response = self._sendCommand("get_actual_joint_positions()")
        return response


    def get_pose(self):
        response = self._sendCommand("get_actual_tcp_pose()")
        return response
