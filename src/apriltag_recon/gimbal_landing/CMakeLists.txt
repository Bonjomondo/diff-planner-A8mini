cmake_minimum_required(VERSION 2.8.3)
project(gimbal_landing)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(PYTHON_EXECUTABLE "/usr/bin/python3.8")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(cv_bridge_DIR "~/Lib/cv_bridge_pkgs/devel/share/cv_bridge/cmake")
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf
    cv_bridge
    std_msgs
    sensor_msgs
    nav_msgs
    apriltag_ros
    mavros
)

catkin_package(
	 INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
#	  DEPENDS system_lib
)

include_directories(
       SYSTEM
		include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
)

add_executable(gimbal_landing
		src/gimbal_landing.cpp
		src/main_node.cpp
)
target_link_libraries(gimbal_landing
		${catkin_LIBRARIES}
		Eigen3::Eigen
)

