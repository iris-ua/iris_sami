
class ArmIF(object):
    """ Interface for arm manipulation """

    def __init__(self):
        self._velocity = 0.5
        self.last_error_msg = ""

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if value > 0.0 and value <= 1.0:
            self._velocity = value
        else:
            raise ValueError("Invalid velocity value. Allowed values are in (0,1]")

    def move_joints(self, joints, velocity):
        raise NotImplementedError

    def move_pose(self, dpose, velocity):
        raise NotImplementedError

    def move_pose_relative(self, pose, velocity):
        raise NotImplementedError


class GripperIF(object):
    """ Interface for gripper control """

    def __init__(self):
        self._velocity = 1.0

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if value > 0.0 and value <= 1.0:
            self._velocity = value
        else:
            raise ValueError("Invalid velocity value. Allowed values are in (0,1]")

    def grip(self):
        raise NotImplementedError

    def release(self):
        raise NotImplementedError

    def get_status(self):
        raise NotImplementedError

    def get_position(self):
        raise NotImplementedError

    def get_force(self):
        raise NotImplementedError

    def set_position(self):
        raise NotImplementedError

    def set_force(self):
        raise NotImplementedError

