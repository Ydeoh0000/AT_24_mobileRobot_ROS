cmake_minimum_required(VERSION 3.0.2)
project(ard_serial_ctrl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
   nodes/ard_serial_ctrl
   scripts/ard_serial_repeater.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
## Testing ##
#############