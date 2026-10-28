cmake_minimum_required(VERSION 3.0.2)
project(multipoint)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

add_message_files(FILES RemoteWaypoint.msg MissionStatus.msg)
add_service_files(FILES MissionCommand.srv)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime nav_msgs quadrotor_msgs roscpp rospy sensor_msgs std_msgs tf
                 visualization_msgs
  DEPENDS EIGEN3
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(multipointplan src/multipointplan.cpp)
add_dependencies(multipointplan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multipointplan yaml-cpp ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/mission_start.test)
  add_rostest(test/remote_mission.test)
  add_rostest(test/remote_cancel.test)
  add_rostest(test/point_roundtrip_planner.test)
  add_rostest(test/automatic_cancel_takeoff.test)
  add_rostest(test/automatic_takeoff_odom_loss.test)
  add_rostest(test/automatic_takeoff_rc_loss.test)
  add_rostest(test/automatic_takeoff_rejected.test)
  add_rostest(test/automatic_cancel_landing.test)
  add_rostest(test/automatic_landing_timeout.test)
  add_rostest(test/automatic_landing_blocked.test)
  add_rostest(test/automatic_roundtrip.test)
  add_rostest(test/automatic_takeoff_corridor_blocked.test)
  add_rostest(test/automatic_position_limit_removed.test)
  add_rostest(test/automatic_controller.test)
  add_rostest(test/automatic_controller_delayed.test)
  add_rostest(test/automatic_roundtrip_delayed.test)
  add_executable(test_mission_guard test/test_mission_guard.cpp)
  target_compile_options(test_mission_guard PRIVATE -UNDEBUG)
  add_test(NAME mission_guard COMMAND test_mission_guard)
  add_executable(test_remote_mission test/test_remote_mission.cpp)
  target_compile_options(test_remote_mission PRIVATE -UNDEBUG)
  add_test(NAME remote_mission COMMAND test_remote_mission)
  catkin_add_nosetests(test/test_gcs_protocol.py)
  catkin_add_nosetests(test/test_gcs_gateway_node.py)
  catkin_add_nosetests(test/test_gcs_map_guard.py)
  catkin_add_nosetests(test/test_a8mini_gimbal_node.py)
  catkin_add_nosetests(test/test_a8mini_detection.py)
  catkin_add_nosetests(test/test_a8mini_capture.py)
endif()

catkin_install_python(
  PROGRAMS scripts/a8mini_gimbal_node.py scripts/a8mini_detection_node.py
           scripts/a8mini_detection.py scripts/a8mini_video.py scripts/a8mini_capture.py
           scripts/a8mini_diagnostics.py scripts/a8mini_labels.py
           scripts/gcs_gateway_node.py scripts/gcs_protocol.py scripts/gcs_map_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS multipointplan
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
