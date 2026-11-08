#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <thread>
#include <chrono>
#include <ctime>
#include <atomic>
#include <iostream>
#include <filesystem>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <opencv2/opencv.hpp>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

class SIYI_ROS_SDK
{
public:
    SIYI_ROS_SDK(void);
    ~SIYI_ROS_SDK() = default;

    void process(void);

private:
    Eigen::Vector3d p, v;
    Eigen::Matrix3d R_c2b, R_b2w;
    bool rev_odom = false;
    bool is_updated = false;
    Eigen::Vector3d land_pose, land_target;
    void pubLandPose(Eigen::Vector3d &pose);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

    ros::Publisher landing_pose_pub;
    ros::Subscriber tag_sub, odom_sub;
};

#endif // MAINWINDOW_H
