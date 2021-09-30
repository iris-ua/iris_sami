#!/usr/bin/env python
import os, time
from math import degrees
import rospy, rospkg, rosparam
from geometry_msgs.msg import Vector3, WrenchStamped
from std_srvs.srv import Trigger
from ur_msgs.srv import SetSpeedSliderFraction
from controller_manager_msgs.srv import ListControllers, SwitchController

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QObject, QThread, pyqtSignal
from python_qt_binding.QtWidgets import QWidget

from iris_sami.msg import ArmInfo
from iris_sami.srv import JointGoalName, RelativeMove, PoseGoal, JointGoal, Status, NoArguments

BASE_DIR = rospkg.RosPack().get_path('iris_sami')
ALIAS_FILE = BASE_DIR + '/yaml/positions.yaml'


class SamiPlugin(Plugin):

    def __init__(self, context):
        super(SamiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Sami Plugin')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('iris_sami'), 'resource', 'SamiPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Sami Plugin UI')

        # Set widget functions
        # Saved Alias Positions
        self.fillAliasLayout()
        self._widget.alias_send.clicked[bool].connect(self.alias_send_button_clicked)
        self._widget.alias_save.clicked[bool].connect(self.alias_save_button_clicked)

        # Thread for robot velocity control       
        self.vel_thread = QThread()
        self.vel_worker = VelocityControl()
        self.vel_worker.moveToThread(self.vel_thread)
        self.vel_thread.started.connect(self.vel_worker.run)
        self.vel_thread.start()

        # Linear Axis Move
        self._widget.x_lin_up.pressed.connect(lambda: self.velocity_move_pressed('lx+'))
        self._widget.x_lin_up.released.connect(self.velocity_move_released)
        self._widget.x_lin_down.pressed.connect(lambda: self.velocity_move_pressed('lx-'))
        self._widget.x_lin_down.released.connect(self.velocity_move_released)
        self._widget.y_lin_up.pressed.connect(lambda: self.velocity_move_pressed('ly+'))
        self._widget.y_lin_up.released.connect(self.velocity_move_released)
        self._widget.y_lin_down.pressed.connect(lambda: self.velocity_move_pressed('ly-'))
        self._widget.y_lin_down.released.connect(self.velocity_move_released)
        self._widget.z_lin_up.pressed.connect(lambda: self.velocity_move_pressed('lz+'))
        self._widget.z_lin_up.released.connect(self.velocity_move_released)
        self._widget.z_lin_down.pressed.connect(lambda: self.velocity_move_pressed('lz-'))
        self._widget.z_lin_down.released.connect(self.velocity_move_released)

        # Angular Axis Move
        self._widget.x_ang_up.pressed.connect(lambda: self.velocity_move_pressed('ax+'))
        self._widget.x_ang_up.released.connect(self.velocity_move_released)
        self._widget.x_ang_down.pressed.connect(lambda: self.velocity_move_pressed('ax-'))
        self._widget.x_ang_down.released.connect(self.velocity_move_released)
        self._widget.y_ang_up.pressed.connect(lambda: self.velocity_move_pressed('ay+'))
        self._widget.y_ang_up.released.connect(self.velocity_move_released)
        self._widget.y_ang_down.pressed.connect(lambda: self.velocity_move_pressed('ay-'))
        self._widget.y_ang_down.released.connect(self.velocity_move_released)
        self._widget.z_ang_up.pressed.connect(lambda: self.velocity_move_pressed('az+'))
        self._widget.z_ang_up.released.connect(self.velocity_move_released)
        self._widget.z_ang_down.pressed.connect(lambda: self.velocity_move_pressed('az-'))
        self._widget.z_ang_down.released.connect(self.velocity_move_released)

        # Relative move service
        self._widget.r_move.clicked[bool].connect(self.rel_move_button_clicked)
        self._widget.r_reset.clicked[bool].connect(self.rel_move_reset_button_clicked)
        # Pose goal service
        self._widget.p_move.clicked[bool].connect(self.pose_button_clicked)
        self._widget.p_reset.clicked[bool].connect(self.pose_reset_button_clicked)
        # Joint goal service
        self._widget.j_move.clicked[bool].connect(self.joints_button_clicked)
        self._widget.j_reset.clicked[bool].connect(self.joints_reset_button_clicked)
        # Gripper services
        self._widget.grip.clicked[bool].connect(self.gripper_button_clicked)
        self._widget.release.clicked[bool].connect(self.release_button_clicked)
        # Controller switcher
        self._widget.switch_controller.clicked[bool].connect(self.switch_controller_button_clicked)
        # Resend robot program
        self._widget.resend_program.clicked[bool].connect(self.resend_program_button_clicked)
        # Zero FT sensor
        self._widget.zero_ft_sensor.clicked[bool].connect(self.zero_ft_sensor_button_clicked)
        # Set Speed Slider
        self._widget.set_speed_slider.clicked[bool].connect(self.set_speed_slider_button_clicked)


        # Status thread for robot information       
        self.status_thread = QThread()
        self.status_worker = StatusDialog()
        self.status_worker.moveToThread(self.status_thread)
        self.status_worker.status_signal.connect(self.status_update)
        self.status_thread.started.connect(self.status_worker.get_status)
        self.status_thread.start()

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        # Add widget to the user interface
        context.add_widget(self._widget)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass


    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


    def fillAliasLayout(self):
        # Fill ComboBox with saved positions
        positions_dict = rosparam.load_file(ALIAS_FILE)[0][0]
        positions = sorted(positions_dict.keys())

        for position in positions:
            self._widget.alias_combo.addItem(position)
    

    def alias_send_button_clicked(self):
        # Call alias service with selected alias position
        alias_pos = self._widget.alias_combo.currentText()

        resp = ''
        rospy.wait_for_service('iris_sami/alias')
        try:
            aliasServ = rospy.ServiceProxy('iris_sami/alias', JointGoalName)
            resp = aliasServ(alias_pos).feedback
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e

        self._widget.com_response.setPlainText(resp)
    

    def alias_save_button_clicked(self):
        # TODO implement save alias function
        pass


    def velocity_move_pressed(self, velocity):
        self.vel_worker.activate(velocity)


    def velocity_move_released(self):
        self.vel_worker.deactivate()


    def status_update(self, data):
        # Slot function that updates GUI with status information
        self._widget.cur_status.setHtml(data)


    def rel_move_button_clicked(self):
        # Relative move service caller with XYZ RPY fields
        x = self._widget.r_x.text()
        y = self._widget.r_y.text()
        z = self._widget.r_z.text()
        roll = self._widget.r_roll.text()
        pitch = self._widget.r_pitch.text()
        yaw = self._widget.r_yaw.text()
        
        rel_move = [float(i) if i != '' else 0 for i in [x, y, z, roll, pitch, yaw]]

        rospy.wait_for_service('iris_sami/move')
        try:
            moveServ = rospy.ServiceProxy('iris_sami/move', RelativeMove)
            resp = moveServ(*rel_move).feedback
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e
        
        self._widget.com_response.setPlainText(resp)
    

    def rel_move_reset_button_clicked(self):
        # Clear all relative move related fields
        self._widget.r_x.clear()
        self._widget.r_y.clear()
        self._widget.r_z.clear()
        self._widget.r_roll.clear()
        self._widget.r_pitch.clear()
        self._widget.r_yaw.clear()


    def pose_button_clicked(self):
        # Pose goal service caller with XYZ RPY fields
        x = self._widget.p_x.text()
        y = self._widget.p_y.text()
        z = self._widget.p_z.text()
        roll = self._widget.p_roll.text()
        pitch = self._widget.p_pitch.text()
        yaw = self._widget.p_yaw.text()
        
        pose = [float(i) if i != '' else 0 for i in [x, y, z, roll, pitch, yaw]]

        rospy.wait_for_service('iris_sami/pose')
        try:
            poseServ = rospy.ServiceProxy('iris_sami/pose', PoseGoal)
            resp = poseServ(*pose).feedback
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e
        
        self._widget.com_response.setPlainText(resp)
    

    def pose_reset_button_clicked(self):
        # Clear all relative move related fields
        self._widget.p_x.clear()
        self._widget.p_y.clear()
        self._widget.p_z.clear()
        self._widget.p_roll.clear()
        self._widget.p_pitch.clear()
        self._widget.p_yaw.clear()

    
    def joints_button_clicked(self):
        # Joints goal service caller with 6 joint fields
        j_s_p = self._widget.j_s_p.text()
        j_s_l = self._widget.j_s_l.text()
        j_elb = self._widget.j_elb.text()
        j_w_1 = self._widget.j_w_1.text()
        j_w_2 = self._widget.j_w_2.text()
        j_w_3 = self._widget.j_w_3.text()
        
        joints = [float(i) if i != '' else 0 for i in [j_s_p, j_s_l, j_elb, j_w_1, j_w_2, j_w_3]]

        rospy.wait_for_service('iris_sami/joints')
        try:
            jointsServ = rospy.ServiceProxy('iris_sami/joints', JointGoal)
            resp = jointsServ(*joints).feedback
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e
        
        self._widget.com_response.setPlainText(resp)
    

    def joints_reset_button_clicked(self):
        # Clear all joints goal related fields
        self._widget.j_s_p.clear()
        self._widget.j_s_l.clear()
        self._widget.j_elb.clear()
        self._widget.j_w_1.clear()
        self._widget.j_w_2.clear()
        self._widget.j_w_3.clear()

        joints = self.status_worker.joints
        
        self._widget.j_s_p.setText(str(joints[0]))
        self._widget.j_s_l.setText(str(joints[1]))
        self._widget.j_elb.setText(str(joints[2]))
        self._widget.j_w_1.setText(str(joints[3]))
        self._widget.j_w_2.setText(str(joints[4]))
        self._widget.j_w_3.setText(str(joints[5]))
    

    def gripper_button_clicked(self):
        # Grip service caller
        rospy.wait_for_service('iris_sami/grip')
        try:
            gripServ = rospy.ServiceProxy('iris_sami/grip', NoArguments)
            resp = gripServ().feedback
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e

        self._widget.com_response.setPlainText(resp)


    def release_button_clicked(self):
        # Release service caller
        rospy.wait_for_service('iris_sami/release')
        try:
            releaseServ = rospy.ServiceProxy('iris_sami/release', NoArguments)
            resp = releaseServ().feedback
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e

        self._widget.com_response.setPlainText(resp)


    def switch_controller_button_clicked(self):
        controllers = ['joint_group_vel_controller', 'scaled_pos_joint_traj_controller']
        status = ''

        # Get active controller
        active_controller = ''
        rospy.wait_for_service('controller_manager/list_controllers')
        try:
            listServ = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
            resp = listServ()
            for controller in resp.controller:
                if controller.name in controllers and controller.state == "running":
                    active_controller = controller.name
        except rospy.ServiceException as e:
            status = "Service call failed: %s" % e

        # Switch Controllers
        if active_controller:
            controllers.remove(active_controller)

            rospy.wait_for_service('controller_manager/switch_controller')
            try:
                switchServ = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
                resp = switchServ(start_controllers=controllers,
                                stop_controllers=[active_controller],
                                strictness=1,
                                start_asap=False,
                                timeout=2.0)
                status = "Switched Controllers: " + controllers[0] + " is active"
            except rospy.ServiceException as e:
                status = "Service call failed: %s" % e
        else:
            status = "No controller is active. There might be a problem with the driver"
        
        self._widget.com_response.setPlainText(status)


    def resend_program_button_clicked(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/resend_robot_program', timeout=1)
            resend = rospy.ServiceProxy('/ur_hardware_interface/resend_robot_program', Trigger)
            if resend().success:
                resp = "Program sucessfyly sent to robot"
            else:
                resp = "Error un resending program to robot"
        except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
            resp = "Service call failed: %s" % e

        self._widget.com_response.setPlainText(resp)


    def zero_ft_sensor_button_clicked(self):
        try:
            rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', timeout=1)
            zero = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor', Trigger)
            if zero().success:
                resp = "FT Sensor sucessfully tared"
            else:
                resp = "Error in taring FT Sensor"
        except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
            resp = "Service call failed: %s" % e

        self._widget.com_response.setPlainText(resp)


    def set_speed_slider_button_clicked(self):
        speed_slider = self._widget.speed_slider.text()
        speed_slider = float(speed_slider)
        
        rospy.wait_for_service('/ur_hardware_interface/set_speed_slider')
        try:
            set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
            resp = set_speed(speed_slider)
            resp = resp.success
        except rospy.ServiceException as e:
            resp = "Service call failed: %s" % e




    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


class VelocityControl(QObject):
    axis_to_vel = {
        's' : [0, 0, 0],
        'lx+' : [1, 0, 0],
        'ly+' : [0, 1, 0],
        'lz+' : [0, 0, 1],
        'lx-' : [-1, 0, 0],
        'ly-' : [0, -1, 0],
        'lz-' : [0, 0, -1],
        'ax+' : [0.5, 0, 0],
        'ay+' : [0, 0.5, 0],
        'az+' : [0, 0, 0.5],
        'ax-' : [-0.5, 0, 0],
        'ay-' : [0, -0.5, 0],
        'az-' : [0, 0, -0.5],
    }

    def __init__(self):
        super(VelocityControl, self).__init__()

        self.active = False
        self.velocity = 's'

        self.wrench_vel_pub = rospy.Publisher('/wrench_velocity', WrenchStamped, queue_size=1)
        

    def activate(self, velocity):
        self.velocity = velocity
        self.active = True


    def deactivate(self):
        self.velocity = 's'
        time.sleep(0.1)
        self.active = False


    def run(self):
        rate = rospy.Rate(500)

        while not rospy.is_shutdown():
            if self.active:
                wrench_vel_msg = WrenchStamped()
                vel = Vector3(*self.axis_to_vel[self.velocity])
                
                if self.velocity[0] == 'l':
                    wrench_vel_msg.wrench.force = vel
                elif self.velocity[0] == 'a':
                    wrench_vel_msg.wrench.torque = vel
                else:
                    wrench_vel_msg.wrench.force = vel
                    wrench_vel_msg.wrench.torque = vel
                
                self.wrench_vel_pub.publish(wrench_vel_msg)
            
            rate.sleep()


class StatusDialog(QObject):
    status_signal = pyqtSignal(object)

    def __init__(self):
        super(StatusDialog, self).__init__()

        self.status = ''
        self.joints = ''
        self.position = ''
        self.orientation = ''
        self.velocity = 0

        rospy.Subscriber('iris_sami/arm_status', ArmInfo, self.status_subscriber)
        

    def status_subscriber(self, data):
        if data.success:
            self.status = data.feedback
            self.joints = [round(x, 3) for x in data.joints]
            pos = data.pose.position
            self.position = [round(x, 5) for x in [pos.x, pos.y, pos.z]]
            ori = data.pose.orientation
            self.orientation = [round(x, 5) for x in [ori.x, ori.y, ori.z, ori.w]]
            self.velocity = data.velocity
    

    def get_status(self):
        # Call status service to obtain information about the robot
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            

            joints_str = '[' + ', '.join(['%.3f (<b>%.1f</b>)' % (x, degrees(x)) for x in self.joints]) + ']'

            status_text = ("<html> \
                            <b>Status:</b> %s<br>\
                            <b>Joints:</b> %s<br>\
                            <b>Position:</b> %s<br>\
                            <b>Orientation:</b> %s<br>\
                            <b>Velocity:</b> %.1f \
                            </html>" % (self.status, joints_str, str(self.position), 
                                        str(self.orientation), self.velocity))
            
            self.status_signal.emit(status_text)
            
            rate.sleep()