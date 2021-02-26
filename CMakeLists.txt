cmake_minimum_required(VERSION 3.5.1)
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
  Actionlist.srv
  PoseGoal.srv
  RelativeMove.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp_v3)
include_directories(${behaviortree_cpp_v3_INCLUDE_DIRS})

add_executable(behavior_tree src/bt/tree.cpp)
target_link_libraries(behavior_tree ${behaviortree_cpp_v3_LIBRARIES})

add_executable(ur10e_tree src/bt/ur10e.cpp)
target_link_libraries(ur10e_tree ${behaviortree_cpp_v3_LIBRARIES} ${catkin_LIBRARIES})

catkin_package()
