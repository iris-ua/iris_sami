cmake_minimum_required(VERSION 2.8.3)
project(iris_sami)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
)

add_service_files(
  FILES
  JointGoal.srv
  PoseGoal.srv
  RelativeMove.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
