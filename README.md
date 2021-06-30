# Simple Arm Manipulation Interface

iris_sami is a wrapper of several motion planning features provided by MoveIt in order to control robotic manipulators.

It was developed and tested with a Univernal Robots UR10e but can be extended for other robotic manipulators.

It is currenlty divided in 4 groups of features:

- **sami - **Group of classes that wrap and simplify MoveIt functions
  - Example of use can be found in scripts/test.py
- **server - **Server that exposes all the funcitonalities provided by sami as a group of ROS Services
  - Info about each service can be obtained with rosservice info /iris_sami/*
- **bt - **Integration of the previous functionalities wiht the BehavioralTree.cpp library for GUI programming of the manipulator
  - Detailed documentation can be found in docs/control_behavior_ur10e.pdf
- **rqt_sami - **GUI implemented using the rqt library to easily call the ROS services exposed by the server
  - Needs the server to be active and can be run with rqt --standalone rqt_sami

### Usage

`roslaunch iris_sami iris_sami.launch`

