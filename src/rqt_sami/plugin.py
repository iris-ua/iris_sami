import os
import rospy, rospkg, rosparam

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
        # Status thread for robot information       
        self.thread = QThread()
        self.worker = StatusDialog()
        self.worker.moveToThread(self.thread)
        self.worker.status_signal.connect(self.status_update)
        self.thread.started.connect(self.worker.get_status)
        self.thread.start()

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

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


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
            self.joints = [round(x, 5) for x in data.joints]
            self.position = data.pose.position
            self.orientation = data.pose.orientation
            self.velocity = data.velocity
    

    def get_status(self):
        # Call status service to obtain information about the robot
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            status_text = ("<html> \
                            <b>Status:</b> %s<br>\
                            <b>Joints:</b> %s<br>\
                            <b>Position:</b> %s<br>\
                            <b>Orientation:</b> %s<br>\
                            <b>Velocity:</b> %f \
                            </html>" % (self.status, str(self.joints), str(self.position), 
                                        str(self.orientation), self.velocity))
            
            self.status_signal.emit(status_text)
            
            rate.sleep()