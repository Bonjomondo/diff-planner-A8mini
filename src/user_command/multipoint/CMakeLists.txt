cmake_minimum_required(VERSION 3.0.2)
project(multipoint)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs quadrotor_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(multipointplan src/multipointplan.cpp)
target_link_libraries(multipointplan yaml-cpp ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_a8mini_gimbal_node.py)
endif()

catkin_install_python(
  PROGRAMS scripts/a8mini_gimbal_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS multipointplan
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
