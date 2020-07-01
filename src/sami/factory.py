
from sami.plugs import moveit, wrgripper

class ArmIFFactory(object):
    @staticmethod
    def arm_with_name(name, options):
        if name == 'ur10e':
            return moveit.MoveItPlug(options)

        return None

class GripperIFFactory(object):
    @staticmethod
    def gripper_with_name(name, options):
        if name == 'cr200-85':
            return wrgripper.CR200Plug(options)

        return None
