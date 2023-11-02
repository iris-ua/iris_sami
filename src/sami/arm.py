import rospy, rosparam, rospkg
import csv, math

from sami.factory import ArmIFFactory


def EzPose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return [x, y, z, roll, pitch, yaw]


class JointPositionAliases():
    def __init__(self, filename):
        self.positions_file = filename
        self.joint_positions = self.load_joint_position_file(self.positions_file)
        if not self.joint_positions:
            raise Exception

        self.error_msg = ""

    def get_joint_configuration(self, position_name):
        try:
            return self.joint_positions[position_name]
        except KeyError:
            raise KeyError

    def load_joint_position_file(self, filename):
        '''Returns the dict with the positions previously saved in a yaml file'''
        try:
            data = rosparam.load_file(filename)
        except rosparam.RosParamException:
            # If not found, check again in the current package
            try:
                filename = rospkg.RosPack().get_path('iris_sami') + '/yaml/' + filename
                data = rosparam.load_file(filename)
            except Exception as e:
                self.error_msg = "Can't load positions from'" + filename + "'\n" + str(e)
                rospy.logerr(self.error_msg)
                return False

        self.joint_positions = data[0][0]
        self.positions_file = filename

        return data[0][0]

    def save_joint_position(self, position_name, joint_configuration):
        ''' Saves current joint configuration to <filename> with name <position_name>. If the provided file doesn't
            exist, it creates it '''
        try:
            hs = open(self.positions_file, "a+")
            hs.write("\n" + position_name + " : " + str(joint_configuration))
            hs.close()

            self.joint_positions.update({position_name: joint_configuration})
        except Exception as e:
            self.error_msg = "Can't save position '" + position_name + "' to '" + self.positions_file + "'\n" + str(e)
            rospy.logerr(self.error_msg)
            return False
        rospy.loginfo("Successfully saved position '" + position_name + "' to '" + self.positions_file + "'")
        return True


class Arm(object):
    def __init__(self, name, **options):
        self.arm_interface = ArmIFFactory.arm_with_name(name, options)

        try:
            self.joint_pos_aliases = JointPositionAliases(options["joint_positions_filename"])
        except Exception:
            self.joint_pos_aliases = None

    def move_joints(self, joints=None, velocity = None):
        """ Move the arm to specified joints. """
        ok = self.arm_interface.move_joints(joints, velocity)
        if not ok:
            rospy.logerr(self.error_msg)
        return ok

    def move_pose(self, pose, velocity = None):
        """ Move the arm to specified pose. """
        ok = self.arm_interface.move_pose(pose, velocity)
        if not ok:
            rospy.logerr(self.error_msg)
        return ok

    def move_pose_relative(self, dpose, velocity = None):
        """ Move the arm relative to the end-effector in a straight trajectory. """
        ok = self.arm_interface.move_pose_relative(dpose, velocity)
        if not ok:
            rospy.logerr(self.error_msg)
        return ok

    def move_pose_relative_world(self, dpose, velocity = None):
        """ Move the arm relative to the world base frame in a straight trajectory """
        ok = self.arm_interface.move_pose_relative_world(dpose, velocity)
        if not ok:
            rospy.logerr(self.error_msg)
        return ok

    def move_chain(self, chain):
        ret = True

        print('Start Move Chain')

        for i, motion in enumerate(chain.mchain):
            if 'q' == motion['type']:
                rospy.loginfo("MoveJoints: {}".format(motion['goal']))
                ret = self.move_joints(motion['goal'], motion['velocity'])
            elif 'p' == motion['type']:
                rospy.loginfo("MovePose: {}".format(motion['goal']))
                ret = self.move_pose(motion['goal'], motion['velocity'])
            elif 'r' == motion['type']:
                rospy.loginfo("MovePoseRelative: {}".format(motion['goal']))
                ret = self.move_pose_relative(motion['goal'], motion['velocity'])
            elif 's' == motion['type']:
                rospy.loginfo("Sleeping: {} seconds".format(motion['goal']))
                rospy.sleep(motion['goal'])
                ret = True
            elif 'a' == motion['type']:
                rospy.loginfo('MoveJointsName: {}'.format(motion['goal']))
                ret = self.move_joints_alias(motion['goal'])
            elif 'f' == motion["type"]:
                rospy.loginfo('Executing custom function')
                ret = motion['goal']()

            if ret == False:
                if motion["type"] != 'n':
                    rospy.logerr(self.error_msg)
                return i+1

        chain.clear()
        return 0

    def load_joint_position_aliases(self, filename):
        ok = self.joint_pos_aliases = JointPositionAliases(filename)
        if not ok:
            self.error_msg = self.joint_pos_aliases.error_msg
            return ok
        return True

    def save_joint_position_alias(self, position_name):
        if self.joint_pos_aliases is None:
            self.error_msg = "No joint position file provided when creating the arm. Try calling the load joint aliases service."
            return False

        ok = self.joint_pos_aliases.save_joint_position(position_name, self.get_joints())
        if not ok:
            self.error_msg = self.joint_pos_aliases.error_msg
            return ok
        return True

    def move_joints_alias(self, position_name):
        if self.joint_pos_aliases is None:
            self.error_msg = "No joint position file provided when creating the arm. Try calling the load joint aliases service."
            return False
        try:
            joints = self.joint_pos_aliases.get_joint_configuration(position_name)
        except KeyError:
            self.error_msg = "Alias not found. Existing positions are: " + str(self.get_joint_position_names())
            return False

        self.move_joints(joints)
        return True

    def get_joints(self):
        return self.arm_interface.get_joints()

    def get_pose(self):
        return self.arm_interface.get_pose()

    def get_joint_position_names(self):
        return self.joint_pos_aliases.joint_positions.keys()

    @property
    def velocity(self):
        return self.arm_interface.velocity

    @velocity.setter
    def velocity(self, value):
        self.arm_interface.velocity = value

    @property
    def error_msg(self):
        return self.arm_interface.last_error_msg

    @error_msg.setter
    def error_msg(self, value):
        self.arm_interface.last_error_msg = value


class ArmMotionChain(object):
    def __init__(self):
        self.mchain = []

    def joints(self, joints, velocity = None):
        self.mchain.append({'goal': joints, 'type': 'q', 'velocity': velocity})
        return self

    def joints_name(self, name, velocity = None):
        self.mchain.append({'goal': name, 'type': 'a', 'velocity': velocity})
        return self

    def pose(self, pose, velocity = None):
        self.mchain.append({'goal': pose, 'type': 'p', 'velocity': velocity})
        return self

    def pose_relative(self, dpose, velocity = None):
        self.mchain.append({'goal': dpose, 'type': 'r', 'velocity': velocity})
        return self

    def sleep(self, seconds):
        self.mchain.append({'goal': seconds, 'type': 's'})
        return self

    def func(self, func):
        self.mchain.append({'goal':func, 'type':'f'})
        return self

    def clear(self):
        self.mchain = []
        return self


