#!/usr/bin/env python
import xmlrpclib

from sami.interface import GripperIF

class CR200Plug(GripperIF):
    def __init__(self, options):
        super(GripperIF, self).__init__()

        connstr = "http://{}:{}/RPC2".format(options['host'], options['port'])
        self.grpc = xmlrpclib.ServerProxy(connstr)

        self.gid = self.grpc.GetGrippers()[0]

        if options['host'] is not 'localhost':
            self.grpc.SetReleaseLimit(self.gid, 1, 80.0)
            self.grpc.SetNoPartLimit(self.gid, 1, 1.0)

    def grip(self):
        return self.grpc.Grip(self.gid, 1)

    def release(self):
        return self.grpc.Release(self.gid, 1)

    def get_status(self):
        return self.grpc.GetState(self.gid)

    def get_position(self):
        return self.grpc.GetPos(self.gid)

    def set_led_preset(self, preset):
        return self.grpc.SetLEDPreset(self.gid, preset)

    def set_led_animation(self, animation):
        return self.grpc.SetLEDAnimation(self.gid, 1, animation)

    def set_led_color(self, color):
        return self.grpc.SetLEDColor(self.gid, 1, color)

    def set_led_speed(self, speed):
        return self.grpc.SetLEDSpeed(self.gid, 1, speed)

    def get_force(self):
        raise NotImplementedError

    def set_position(self):
        raise NotImplementedError

    def set_force(self):
        raise NotImplementedError

