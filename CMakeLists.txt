cmake_minimum_required(VERSION 2.8.3)
project(iris_sami)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

#add_message_files(
#  FILES
#)

add_service_files(
  FILES
  Status.srv
  Velocity.srv
  NoArguments.srv
  JointGoal.srv
  JointGoalName.srv
  LoadJointGoalName.srv
  SaveJointGoalName.srv
  PoseGoal.srv
  RelativeMove.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()
