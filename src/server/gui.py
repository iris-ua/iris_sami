#!/usr/bin/env python
import rospkg, rospy, rosparam, gi, sys, signal
from math import radians, degrees
from std_srvs.srv import Trigger

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib

from iris_sami.srv import JointGoalName, RelativeMove, PoseGoal, JointGoal, Status, NoArguments

BASE_DIR = rospkg.RosPack().get_path('iris_sami')
ALIAS_FILE = BASE_DIR + '/yaml/positions.yaml'


class AliasBox(Gtk.Box):
    def __init__(self):
        Gtk.Box.__init__(self, spacing=6)

        positions_dict = rosparam.load_file(ALIAS_FILE)[0][0]
        positions = sorted(positions_dict.keys())

        alias_combo = Gtk.ComboBoxText()
        alias_combo.set_entry_text_column(0)
        alias_combo.connect("changed", self.on_alias_combo_changed)

        for position in positions:
            alias_combo.append_text(position)

        alias_combo.set_active(0)
        self.active_alias = positions[0]
        self.pack_start(alias_combo, True, True, 0)

        self.alias_button = Gtk.Button(label="Send to Position")
        self.alias_button.connect("clicked", self.on_alias_button_clicked)
        self.pack_start(self.alias_button, False, False, 0)


    def on_alias_combo_changed(self, combo):
        self.active_alias = combo.get_active_text()


    def on_alias_button_clicked(self, widget):
        rospy.wait_for_service('iris_sami/alias')
        try:
            aliasServ = rospy.ServiceProxy('iris_sami/alias', JointGoalName)
            aliasServ(self.active_alias)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class MoveBox(Gtk.Box):
    def __init__(self):
        Gtk.Box.__init__(self, spacing=6)

        self.entries_labels = ['X', 'Y', 'Z', 'R', 'P', 'Y']
        self.entries_widgets = []

        for entry in self.entries_labels:
            label = Gtk.Label(entry)
            input = Gtk.Entry()
            input.set_text("0.0")
            input.set_width_chars(5)
            self.entries_widgets.append(input)
            self.pack_start(label, False, False, 0)
            self.pack_start(input, True, True, 0)
        
        self.reset_button = Gtk.Button(label="Reset")
        self.reset_button.connect("clicked", self.on_reset_button_clicked)
        self.pack_start(self.reset_button, False, False, 0)

        self.move_button = Gtk.Button(label="Move")
        self.move_button.connect("clicked", self.on_move_button_clicked)
        self.pack_start(self.move_button, False, False, 0)

    
    def on_reset_button_clicked(self, widget):
        for input in self.entries_widgets:
            input.set_text("0.0")


    def on_move_button_clicked(self, widget):
        pose = []
        for input in self.entries_widgets[:3]:
            pose += [float(input.get_text())]
        
        for input in self.entries_widgets[3:]:
            pose += [radians(float(input.get_text()))]
        
        rospy.wait_for_service('iris_sami/move')
        try:
            moveServ = rospy.ServiceProxy('iris_sami/move', RelativeMove)
            moveServ(*pose)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class PoseBox(Gtk.Box):
    def __init__(self):
        Gtk.Box.__init__(self, spacing=6)

        self.entries_labels = ['X', 'Y', 'Z', 'R', 'P', 'Y']
        self.entries_widgets = []

        for entry in self.entries_labels:
            label = Gtk.Label(entry)
            input = Gtk.Entry()
            input.set_text("0.0")
            input.set_width_chars(5)
            self.entries_widgets.append(input)
            self.pack_start(label, False, False, 0)
            self.pack_start(input, True, True, 0)
        
        self.reset_button = Gtk.Button(label="Reset")
        self.reset_button.connect("clicked", self.on_reset_button_clicked)
        self.pack_start(self.reset_button, False, False, 0)

        self.move_button = Gtk.Button(label="Pose")
        self.move_button.connect("clicked", self.on_move_button_clicked)
        self.pack_start(self.move_button, False, False, 0)

    
    def on_reset_button_clicked(self, widget):
        for input in self.entries_widgets:
            input.set_text("0.0")


    def on_move_button_clicked(self, widget):
        move = []
        for input in self.entries_widgets[:3]:
            move += [float(input.get_text())]
        
        for input in self.entries_widgets[3:]:
            move += [radians(float(input.get_text()))]
        
        rospy.wait_for_service('iris_sami/pose')
        try:
            moveServ = rospy.ServiceProxy('iris_sami/pose', PoseGoal)
            moveServ(*move)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class JointsBox(Gtk.Box):
    def __init__(self):
        Gtk.Box.__init__(self, spacing=6)

        self.entries_labels = ['S_P', 'S_L', 'E', 'W_1', 'W_2', 'W_3']
        self.entries_widgets = []

        for entry in self.entries_labels:
            label = Gtk.Label(entry)
            input = Gtk.Entry()
            input.set_text("0.0")
            input.set_width_chars(5)
            self.entries_widgets.append(input)
            self.pack_start(label, False, False, 0)
            self.pack_start(input, True, True, 0)
        
        self.reset_button = Gtk.Button(label="Reset")
        self.reset_button.connect("clicked", self.on_reset_button_clicked)
        self.pack_start(self.reset_button, False, False, 0)

        self.move_button = Gtk.Button(label="Joints")
        self.move_button.connect("clicked", self.on_move_button_clicked)
        self.pack_start(self.move_button, False, False, 0)

    
    def on_reset_button_clicked(self, widget):
        rospy.wait_for_service('iris_sami/status')
        try:
            statusServ = rospy.ServiceProxy('iris_sami/status', Status)
            resp = statusServ()
            for i in range(len(self.entries_widgets)):
                self.entries_widgets[i].set_text('%.1f' % degrees(resp.joints[i]))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def on_move_button_clicked(self, widget):
        joints = []
        for input in self.entries_widgets:
            joints += [radians(float(input.get_text()))]
        
        rospy.wait_for_service('iris_sami/joints')
        try:
            moveServ = rospy.ServiceProxy('iris_sami/joints', JointGoal)
            moveServ(*joints)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class GripperBox(Gtk.Box):
    def __init__(self):
        Gtk.Box.__init__(self, spacing=6)

        self.pack_start(Gtk.Box(), True, True, 0)

        self.grip_button = Gtk.Button(label='Grip')
        self.grip_button.connect("clicked", self.on_grip_button_clicked)
        self.pack_start(self.grip_button, False, False, 0)

        self.release_button = Gtk.Button(label='Release')
        self.release_button.connect("clicked", self.on_release_button_clicked)
        self.pack_start(self.release_button, False, False, 0)

        self.pack_start(Gtk.Box(), True, True, 0)


    def on_grip_button_clicked(self, widget):
        rospy.wait_for_service('iris_sami/grip')
        try:
            gripServ = rospy.ServiceProxy('iris_sami/grip', NoArguments)
            gripServ()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def on_release_button_clicked(self, widget):
        rospy.wait_for_service('iris_sami/release')
        try:
            releaseServ = rospy.ServiceProxy('iris_sami/release', NoArguments)
            releaseServ()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class FTSensorBox(Gtk.Box):
    def __init__(self):
        Gtk.Box.__init__(self, spacing=6)

        self.pack_start(Gtk.Box(), True, True, 0)

        self.zero_button = Gtk.Button(label='Zero FT Sensor')
        self.zero_button.connect("clicked", self.on_zero_button_clicked)
        self.pack_start(self.zero_button, False, False, 0)

        self.pack_start(Gtk.Box(), True, True, 0)


    def on_zero_button_clicked(self, widget):
        rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor')
        try:
            zeroServ = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)
            zeroServ()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class MyWindow(Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self, title="iris_sami GUI control")

        self.set_border_width(10)

        vbox = Gtk.VBox(spacing=10)
        # Moiton Planning
        vbox.pack_start(Gtk.Label("Move Robot to a saved custom position"), False, False, 0)
        vbox.pack_start(AliasBox(), False, False, 0)
        vbox.pack_start(Gtk.Label("Move Robot in the EE orientation"), False, False, 0)
        vbox.pack_start(MoveBox(), False, False, 0)
        vbox.pack_start(Gtk.Label("Move Robot to global pose"), False, False, 0)
        vbox.pack_start(PoseBox(), False, False, 0)
        vbox.pack_start(Gtk.Label("Set custom values for Robot's Joints"), False, False, 0)
        vbox.pack_start(JointsBox(), False, False, 0)
        # Controls
        cbox = Gtk.Box(spacing=10)
        cbox.pack_start(Gtk.Box(), True, True, 0)
        gbox = Gtk.VBox(spacing=10)
        gbox.pack_start(Gtk.Label("Gripper Control"), False, False, 0)
        gbox.pack_start(GripperBox(), False, False, 0)
        cbox.pack_start(gbox, False, False, 0)
        fbox = Gtk.VBox(spacing=10)
        fbox.pack_start(Gtk.Label("FT Sensor Control"), False, False, 0)
        fbox.pack_start(FTSensorBox(), False, False, 0)
        cbox.pack_start(fbox, False, False, 0)
        cbox.pack_start(Gtk.Box(), True, True, 0)
        vbox.pack_start(cbox, False, False, 0)
        vbox.pack_start(Gtk.Box(), True, True, 0)

        self.add(vbox)


def main():
    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, Gtk.main_quit)

    win = MyWindow()
    win.connect("destroy", Gtk.main_quit)
    win.show_all()
    Gtk.main()


if __name__ == '__main__':
    main()