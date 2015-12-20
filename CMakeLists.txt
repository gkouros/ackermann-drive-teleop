cmake_minimum_required(VERSION 2.8.3)
project(ackermann_drive_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ackermann_msgs
)


catkin_package(
  INCLUDE_DIRS scripts
  # LIBRARIES
  CATKIN_DEPENDS
    rospy
    ackermann_msgs
  # DEPENDS
)

# catkin_install_python(PROGRAMS
  # scripts/ackermann_drive_keyop.py
  # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
