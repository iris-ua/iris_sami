#!/usr/bin/env python3

#python2.7 from httplib import CannotSendHeader, CannotSendRequest, ResponseNotReady
#python2.7 import xmlrpclib

from http.client import CannotSendHeader, CannotSendRequest, ResponseNotReady
from xmlrpc import client

from sami.interface import GripperIF

class CR200Plug(GripperIF):
    def __init__(self, options):
        super(GripperIF, self).__init__()

        connstr = "http://{}:{}/RPC2".format(options['host'], options['port'])
        #python2.7 self.grpc = xmlrpclib.ServerProxy(connstr)
        self.grpc = client.ServerProxy(connstr)

        self.gid = self.grpc.GetGrippers()[0]
        # repo self.state = self.grpc.GetState(self.gid)
        #self.state = self.grpc.GetState(self.gid, 0)   # HACK for solving idx error in reply (simulation : fake_gripper)
        self.state = self.grpc.GetState(self.gid)      
        self.position = self.grpc.GetPos(self.gid)

        if options['host'] is not 'localhost':
            self.grpc.SetReleaseLimit(self.gid, 1, 80.0)
            self.grpc.SetNoPartLimit(self.gid, 1, 1.0)

    def grip(self):
        return self.grpc.Grip(self.gid, 1)

    def release(self):
        return self.grpc.Release(self.gid, 1)

    def get_status(self):
        new_state = 0
        try:
            new_state = self.grpc.GetState(self.gid)
        except (CannotSendHeader, CannotSendRequest, ResponseNotReady) as e:
            print('Error obtaining gripper state', e)
            new_state = self.state
        
        self.state = new_state        
        return self.state

    def get_position(self):
        new_pos = 0
        try:
            new_pos = self.grpc.GetPos(self.gid)
        except (CannotSendHeader, CannotSendRequest, ResponseNotReady) as e:
            print('Error obtaining gripper position', e)
            new_pos = self.pos
        
        self.pos = new_pos        
        return self.pos

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

