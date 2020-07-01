
import rospy

from sami.factory import ArmIFFactory


def EzPose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return [x, y, y, roll, pitch, yaw]


class Arm(object):
    def __init__(self, name, **options):
        self.arm_interface = ArmIFFactory.arm_with_name(name, options)

    def move_joints(self, joints, velocity = None):
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

