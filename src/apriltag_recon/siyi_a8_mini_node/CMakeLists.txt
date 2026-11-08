cmake_minimum_required(VERSION 3.0.2)
project(siyi_a8_mini_node)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/image_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
