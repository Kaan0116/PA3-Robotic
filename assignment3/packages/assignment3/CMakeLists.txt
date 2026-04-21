cmake_minimum_required(VERSION 2.8.3)
project(assignment3)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  duckietown_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/navigator_node.py
  src/aruco_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES src/astar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
