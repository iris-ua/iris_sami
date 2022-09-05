
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

    def set_led_preset(self, preset):
        return self.gripper_interface.set_led_preset(preset)

    def set_led_animation(self, animation):
        return self.gripper_interface.set_led_animation(animation)

    def set_led_color(self, color):
        return self.gripper_interface.set_led_color(color)
    
    def set_led_speed(self, speed):
        return self.gripper_interface.set_led_speed(speed)

    def get_force(self):
        raise NotImplementedError

    def set_position(self):
        raise NotImplementedError

    def set_force(self):
        raise NotImplementedError


