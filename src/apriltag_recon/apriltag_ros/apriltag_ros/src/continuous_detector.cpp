/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit()
{
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
    draw_tag_detections_image_ =
        getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false);
    it_ = std::shared_ptr<image_transport::ImageTransport>(
        new image_transport::ImageTransport(nh));

    std::string transport_hint;
    pnh.param<std::string>("transport_hint", transport_hint, "raw");

    // Read camera intrinsics from private parameters
    pnh.param("fx", fx_, 0.0);
    pnh.param("fy", fy_, 0.0);
    pnh.param("cx", cx_, 0.0);
    pnh.param("cy", cy_, 0.0);

    // 2. Camera intrinsic matrix K (1280x720).
    K = cv::Matx33d(723.4472235974023,
                    0.0,
                    626.6096801167150,
                    0.0,
                    723.1915950575013,
                    381.5852783733815,
                    0.0,
                    0.0,
                    1.0);

    // Distortion coefficients: k1, k2, p1, p2, k3.
    static const cv::Mat D = (cv::Mat_<double>(1, 5) << -0.1119509916483838,
                              0.09485696017034999,
                              -0.00057886515484381,
                              -0.0027310943341364224,
                              0.0);

    // Rectified projection matrix P (left 3x3 part).
    static const cv::Matx33d K_rect(715.9202270507812,
                                    0.0,
                                    623.3036087133878,
                                    0.0,
                                    714.3930053710938,
                                    379.7094075857240,
                                    0.0,
                                    0.0,
                                    1.0);

    cv::initUndistortRectifyMap(K,
                                D,
                                cv::Mat(),
                                K_rect,
                                cv::Size(1280, 720),
                                CV_16SC2,
                                map1_,
                                map2_);

    int queue_size;
    pnh.param<int>("queue_size", queue_size, 1);
    camera_image_subscriber_ =
        it_->subscribe("image_rect",
                       queue_size,
                       &ContinuousDetector::imageCallback,
                       this,
                       image_transport::TransportHints(transport_hint));
    tag_detections_publisher_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
    if (draw_tag_detections_image_)
        tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);

    refresh_params_service_ =
        pnh.advertiseService("refresh_tag_params",
                             &ContinuousDetector::refreshParamsCallback,
                             this);
}

void ContinuousDetector::refreshTagParameters()
{
    // Resetting the tag detector will cause a new param server lookup
    // So if the parameters have changed (by someone/something),
    // they will be updated dynamically
    std::scoped_lock<std::mutex> lock(detection_mutex_);
    ros::NodeHandle &pnh = getPrivateNodeHandle();
    tag_detector_.reset(new TagDetector(pnh));
}

bool ContinuousDetector::refreshParamsCallback(std_srvs::Empty::Request &req,
                                               std_srvs::Empty::Response &res)
{
    refreshTagParameters();
    return true;
}

void ContinuousDetector::imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
{
    std::scoped_lock<std::mutex> lock(detection_mutex_);
    // Lazy updates:
    // When there are no subscribers _and_ when tf is not published,
    // skip detection.
    if (tag_detections_publisher_.getNumSubscribers() == 0 &&
        tag_detections_image_publisher_.getNumSubscribers() == 0 &&
        !tag_detector_->get_publish_tf())
    {
        // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
        return;
    }

    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the iamge
    try
    {
        cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 3. Rectify the raw image.
    cv::Mat rectified_image;
    cv::remap(cv_image_->image, rectified_image, map1_, map2_, cv::INTER_LINEAR);
    cv_image_->image = rectified_image;

    // Construct CameraInfo in memory; no camera_info topic needed.
    sensor_msgs::CameraInfoPtr camera_info(new sensor_msgs::CameraInfo());
    camera_info->header = image_rect->header;
    camera_info->width = image_rect->width;
    camera_info->height = image_rect->height;
    // Assume the input image is already rectified.
    camera_info->distortion_model = "plumb_bob";

    std::copy(K.val, K.val + 9, camera_info->K.begin());
    camera_info->D = {-0.1119509916483838,
                      0.09485696017034999,
                      -0.00057886515484381,
                      -0.0027310943341364224,
                      0.0};
    camera_info->R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    // Projection matrix P.
    camera_info->P = {715.9202270507812,
                      0.0,
                      623.3036087133878,
                      0.0,
                      0.0,
                      714.3930053710938,
                      379.7094075857240,
                      0.0,
                      0.0,
                      0.0,
                      1.0,
                      0.0};
    sensor_msgs::CameraInfoConstPtr camera_info_const = camera_info;

    // Publish detected tags in the image by AprilTag 2
    AprilTagDetectionArray detections =
        tag_detector_->detectTags(cv_image_, camera_info_const);
    tag_detections_publisher_.publish(detections);

    // Publish the camera image overlaid by outlines of the detected tags and
    // their payload values
    if (draw_tag_detections_image_)
    {
        tag_detector_->drawDetections(cv_image_);
        tag_detector_->drawAxes(cv_image_, camera_info_const, detections);
        tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
}

} // namespace apriltag_ros
