
import xmlrpclib

from sami.interface import GripperIF

class CR200Plug(GripperIF):
    def __init__(self, options):
        super(GripperIF, self).__init__()

        connstr = "http://{}:{}/RPC2".format(options['host'], options['port'])
        self.grpc = xmlrpclib.ServerProxy(connstr)

        self.gid = self.grpc.GetGrippers()[0]

    def grip(self):
        return self.grpc.Grip(self.gid, 1)

    def release(self):
        return self.grpc.Release(self.gid, 1)

    def get_status(self):
        return self.grpc.GetState(self.gid, 1)

    def get_position(self):
        return self.grpc.GetPos(self.gid)

    def get_force(self):
        raise NotImplementedError

    def set_position(self):
        raise NotImplementedError

    def set_force(self):
        raise NotImplementedError

