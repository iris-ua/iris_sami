
import rospy

from sami.factory import GripperIFFactory

class Gripper(object):
    def __init__(self, name, **options):
        self.gripper_interface = GripperIFFactory.gripper_with_name(name, options)

    def grip(self):
        return self.gripper_interface.grip()

    def release(self):
        return self.gripper_interface.release()

    def get_status(self):
        return self.gripper_interface.get_status()

    def get_position(self):
        return self.gripper_interface.get_position()

    def get_force(self):
        raise NotImplementedError

    def set_position(self):
        raise NotImplementedError

    def set_force(self):
        raise NotImplementedError


