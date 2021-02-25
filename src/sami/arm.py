
import rospy, rosparam, rospkg

from sami.factory import ArmIFFactory


def EzPose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return [x, y, z, roll, pitch, yaw]


class Arm(object):
    def __init__(self, name, **options):
        self.arm_interface = ArmIFFactory.arm_with_name(name, options)


        ''' Positions filename can be passed through the options argument fields. If this is done, then all functions regarding
            saving or loading joint position reference names use this variable instead if the function parameter is not provided'''
        
        try:
            self.positions_file = options["joint_positions_filename"] 
            self.joint_positions = self.load_joint_position_file()
        except Exception:
            self.positions_file = None
            self.joint_positions = {}

    def move_joints(self, joints=None, velocity = None, joint_position_name=None):
        """ Move the arm to specified joints. """

        try:
            joints = self.joint_positions[joint_position_name]
        except KeyError:
            rospy.logerr("Joint position name not found. Please provide an existing position name or a <joints> parameter input.")
            return 
        
        if joints is None:
            rospy.logerr("No joint configuration was provided.")
            return

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
        for i, motion in enumerate(chain.mchain):
            if 'q' in motion['type']:
                rospy.loginfo("MoveJoints: {}".format(motion['goal']))
                ret = self.move_joints(motion['goal'], motion['velocity'])
            elif 'p' is motion['type']:
                rospy.loginfo("MovePose: {}".format(motion['goal']))
                ret = self.move_pose(motion['goal'], motion['velocity'])
            elif 'r' is motion['type']:
                rospy.loginfo("MovePoseRelative: {}".format(motion['goal']))
                ret = self.move_pose_relative(motion['goal'], motion['velocity'])
            elif 's' is motion['type']:
                rospy.loginfo("Sleeping: {} seconds".format(motion['goal']))
                rospy.sleep(motion['goal'])
                ret = True

            if ret == False:
                rospy.logerr(self.error_msg)
                return i+1

        chain.clear()
        return 0

    def get_joints(self):
        return self.arm_interface.get_joints()

    def get_pose(self):
        return self.arm_interface.get_pose()

    def get_joint_position_names(self):
        return self.joint_positions.keys()

    def load_joint_position_file(self, filename=None):
        '''Returns tuple with keys of all possible positions and the dict with the positions previously
       saved in a yaml file'''

        if filename is None:
            filename = self.positions_file
            if filename is None:
                rospy.logerr("No filename provided as input. No filename available in the arm context.")
                return 
        
        try:
            data = rosparam.load_file(filename)
        except rosparam.RosParamException:
            # If not found, check again in the current package
            try:
                filename = rospkg.RosPack().get_path('iris_sami') + '/yaml/' + filename
                data = rosparam.load_file(filename)
            except Exception as e:

                rospy.logerr("Can't load positions from'" + filename + "'")
                rospy.logerr(e)
                return

        self.joint_positions = data[0][0]
        self.positions_file = filename

        return data[0][0]


    def save_joint_position(self, position_name, filename=None):
        ''' Saves current joint configuration to <filename> with name <position_name>. If the provided file doesn't
            exist, it creates it '''

        if filename is None:
            filename = self.positions_file
            if filename is None:
                rospy.logerr("No filename provided as input. No filename available in the arm context.")
                return

        try:
            hs = open(filename, "a+")
            hs.write("\n" + position_name + " : " + str(self.get_joints()))
            hs.close()

            self.joint_positions.update({position_name: self.get_joints()})
        except Exception as e:
            rospy.logerr("Can't save position '" + position_name + "' to '" + filename + "'")
            rospy.logerr(e)
            return 
        rospy.loginfo("Successfully saved position '" + position_name + "' to '" + filename + "'")

    @property
    def velocity(self):
        return self.arm_interface.velocity

    @velocity.setter
    def velocity(self, value):
        self.arm_interface.velocity = value

    @property
    def error_msg(self):
        return self.arm_interface.last_error_msg

class ArmMotionChain(object):
    def __init__(self):
        self.mchain = []

    def joints(self, joints, velocity = None):
        self.mchain.append({'goal': joints, 'type': 'q', 'velocity': velocity})
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

    def clear(self):
        self.mchain = []
        return self

