#!/usr/bin/env python3
import rospy, socket
from ur_msgs.srv import SetSpeedSliderFraction

from sami.arm import Arm
from sami.gripper import Gripper
from iris_sami.msg import ArmInfo, GripperInfo
from iris_sami.srv import Status, Velocity, JointGoal, JointGoalName, SaveJointGoalName, Actionlist, \
    LoadJointGoalName, PoseGoal, RelativeMove, NoArguments

arm = None
gripper_send = None
gripper_recv = None

arm_status_pub = None
gripper_status_pub = None

def isOpen(ip,port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(1)
        s.connect((ip, int(port)))
        s.shutdown(2)
        return True
    except Exception as e:
        print(e)
        return False


def arm_status_topic(event=None):
    joints = arm.get_joints()
    pose = arm.get_pose()
    velocity = arm.velocity
    
    arm_status_pub.publish(ArmInfo(*[True, 'Arm is operational', joints, pose, velocity]))


def gripper_status_topic(event=None):
    gripped = gripper_recv.get_status() == 8
    has_object = gripped and gripper_recv.get_position() > 27

    gripper_status_pub.publish(GripperInfo(*[True, 'Gripper is operational', gripped, has_object]))


def info_srv(req):
    joints = arm.get_joints()
    pose = arm.get_pose()
    velocity = arm.velocity
    try:
        gripped = gripper_recv.get_status() == 8
        has_object = gripped and gripper_recv.get_position() > 27
    except AttributeError:
        gripped = False
        has_object = False
    
    return [True, 'Arm is operational', joints, pose, velocity, gripped, has_object]


def velocity_srv(req):
    rospy.loginfo('Velocity service called with value ' + str(req.velocity))
    arm.velocity = req.velocity
    if arm.velocity != req.velocity:
        return [False, 'An error has occured']
    return [True, 'Velocity set to ' + str(req.velocity)]


def move_joint_srv(req):
    rospy.loginfo('Joint Goal service called with values\n' + str(req))
    joints = [req.shoulder_pan, req.shoulder_lift, req.elbow, req.wrist_1, req.wrist_2, req.wrist_3]
    ok = arm.move_joints(joints)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Moved Robot to ' + str(joints)]


def move_joint_name_srv(req):
    rospy.loginfo('Alias Position service called with name ' + str(req))
    pos_name = req.name
    ok = arm.move_joints_alias(pos_name)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Moved Robot to \'' + pos_name + '\' pose']


def save_joint_name_srv(req):
    position = req.name
    ok = arm.save_joint_position_alias(position)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Saved current pose as \'' + str(position) + '\'']


def load_joint_name_srv(req):
    filename = req.joint_positions_file
    ok = arm.load_joint_position_aliases(filename)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Currently loaded \'' + filename + '\'as positions file']


def actionlist_srv(req):
    filename = req.actionlist_filename
    ok = arm.execute_actionlist(filename)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Successfuly executed \'' + filename + '\' actionlist']


def move_pose_srv(req):
    rospy.loginfo('Pose Goal service called with pose\n' + str(req))
    pose = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    ok = arm.move_pose(pose)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Moved Robot to pose ' + str(pose)]


def move_pose_relative_srv(req):
    rospy.loginfo('Relative Move service called with values\n' + str(req))
    pose = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    ok = arm.move_pose_relative(pose)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Moved Robot relatively in ' + str(pose)]


def move_pose_relative_world_srv(req):
    rospy.loginfo('Relative World Move service called with values\n' + str(req))
    pose = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    ok = arm.move_pose_relative_world(pose)
    if not ok:
        return [False, arm.error_msg]
    return [True, 'Moved Robot World relatively in ' + str(pose)]


def grip_srv(req):
    rospy.loginfo('Grip service called')
    result = gripper_send.grip()
    if result:
        return [False, 'An error has ocurred']
    return [True, 'Gripper is Closed']


def release_srv(req):
    rospy.loginfo('Release service called')
    result = gripper_send.release()
    if result:
        return [False, 'An error has ocurred']
    return [True, 'Gripper is Opened']


def main():
    rospy.init_node('sami_server', anonymous=True)

    global arm, gripper_send, gripper_recv
    # Connect to arm
    arm = Arm('ur10e_moveit', group='manipulator', joint_positions_filename="positions.yaml")
    arm.velocity = 0.2

    rospy.loginfo('Robot is ready to receive commands')

    # Arm service registration
    rospy.Service('/iris_sami/status', Status, info_srv)
    rospy.Service('/iris_sami/velocity', Velocity, velocity_srv)
    rospy.Service('/iris_sami/joints', JointGoal, move_joint_srv)
    rospy.Service('/iris_sami/alias', JointGoalName, move_joint_name_srv)
    rospy.Service('/iris_sami/save_alias', SaveJointGoalName, save_joint_name_srv)
    rospy.Service('/iris_sami/load_joints_alias', LoadJointGoalName, load_joint_name_srv)
    rospy.Service('/iris_sami/actionlist', Actionlist, actionlist_srv)
    rospy.Service('/iris_sami/pose', PoseGoal, move_pose_srv)
    rospy.Service('/iris_sami/move', RelativeMove, move_pose_relative_srv)
    rospy.Service('/iris_sami/move_world', RelativeMove, move_pose_relative_world_srv)

    # Temporary set speed slider
    try:
        rospy.wait_for_service('/ur_hardware_interface/set_speed_slider', timeout=2)
        set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
        set_speed(0.3)
    except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
        rospy.logerr("Service call failed: %s" % e)
    
    # Connect to Gripper
    if isOpen('10.1.0.2', 44221):
        print('Port 10.1.0.2 is open')
        try:
            gripper_send = Gripper('cr200-85', host='10.1.0.2', port=44221)
            gripper_recv = Gripper('cr200-85', host='10.1.0.2', port=44221)
        except Exception as e:
            rospy.logwarn('Cant connect to real Gripper - %s' % e)
    elif isOpen('localhost', 44221):
        print('Port localhost is open')
        try:
            gripper_send = Gripper('cr200-85', host='localhost', port=44221)
            gripper_recv = Gripper('cr200-85', host='localhost', port=44221)
        except Exception as e:
            rospy.logwarn('Cant connect to fake gripper - %s' % e)
    else:
        rospy.logwarn('Cant connect to any Gripper')

    # Gripper service registration
    rospy.Service('/iris_sami/grip', NoArguments, grip_srv)
    rospy.Service('/iris_sami/release', NoArguments, release_srv)

    # Status Publishers
    global arm_status_pub, gripper_status_pub
    arm_status_pub = rospy.Publisher('iris_sami/arm_status', ArmInfo, queue_size=1)
    rospy.Timer(rospy.Duration(1.0/500.0), arm_status_topic)

    gripper_status_pub = rospy.Publisher('iris_sami/gripper_status', GripperInfo, queue_size=1)
    rospy.Timer(rospy.Duration(1.0/50.0), gripper_status_topic)
         
    rospy.spin()


if __name__ == '__main__':
    main()
