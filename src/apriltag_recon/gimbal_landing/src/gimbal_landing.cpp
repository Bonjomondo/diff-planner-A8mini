#include "gimbal_landing.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <cmath>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <mavros_msgs/ExtendedState.h>

using SteadyClock = std::chrono::steady_clock;
using namespace std::chrono_literals;

enum LandingCameraState
{
    IDLE,
    MOVING_DOWN,
    MOVE_LANDING_POS,
    LANDING,
};

int land_state;
int g_state = IDLE;
std::atomic<bool> g_landing_requested{false};
ros::Subscriber g_landing_subscriber, extended_state_sub;
ros::Publisher g_status_publisher;

std::string g_last_status;

uint16_t siyiCrc16(const uint8_t *data, std::size_t length)
{
    // SIYI SDK CRC: CRC-16/CCITT, polynomial 0x1021, initial value 0.
    uint16_t crc = 0;
    for (std::size_t i = 0; i < length; ++i)
    {
        const uint8_t index = static_cast<uint8_t>(data[i] ^ ((crc >> 8) & 0xFF));
        uint16_t table_value = static_cast<uint16_t>(index) << 8;
        for (int bit = 0; bit < 8; ++bit)
        {
            if (table_value & 0x8000)
                table_value = static_cast<uint16_t>((table_value << 1) ^ 0x1021);
            else
                table_value = static_cast<uint16_t>(table_value << 1);
        }
        crc = static_cast<uint16_t>((crc << 8) ^ table_value);
    }
    return crc;
}

int16_t readInt16LE(const uint8_t *data)
{
    const uint16_t value =
        static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    return static_cast<int16_t>(value);
}

class SiyiGimbalUdp
{
public:
    SiyiGimbalUdp() = default;

    ~SiyiGimbalUdp()
    {
        closeSocket();
    }

    bool initialize(const std::string &ip, int port)
    {
        closeSocket();
        socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0)
        {
            ROS_ERROR("Failed to create SIYI UDP socket: %s", std::strerror(errno));
            return false;
        }

        const int flags = fcntl(socket_fd_, F_GETFL, 0);
        if (flags >= 0)
            fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        std::memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, ip.c_str(), &server_addr_.sin_addr) != 1)
        {
            ROS_ERROR("Invalid SIYI gimbal IP address: %s", ip.c_str());
            closeSocket();
            return false;
        }

        ROS_INFO("SIYI gimbal UDP target: %s:%d", ip.c_str(), port);
        return true;
    }

    bool sendOneKeyPosition(uint8_t position)
    {
        // A8 mini SDK CMD 0x08:
        // 1 = one-key return, 2 = center-and-down, 3 = center, 4 = down.
        const std::vector<uint8_t> payload{position};
        return sendPacket(0x08, payload);
    }

    enum
    {
        LOCK_MODE = 0x03,
        FOLLOW_MODE,
        FPV_MODE,
    };
    bool setMode(uint8_t mode)
    {
        // A8 mini SDK CMD 0x0C:
        // 0x03 = Lock Mode, 0x04 = Follow Mode, 0x05 = FPV Mode
        const std::vector<uint8_t> payload{mode};
        return sendPacket(0x0C, payload);
    }

    // Non-blocking: request attitude periodically and consume any available replies.
    void updateAttitude(SteadyClock::time_point now)
    {
        if (socket_fd_ < 0)
            return;

        if (now >= next_attitude_request_)
        {
            sendPacket(0x0D, {}); // Request gimbal attitude.
            next_attitude_request_ = now + std::chrono::milliseconds(200);
        }

        for (;;)
        {
            uint8_t buffer[256];
            const ssize_t received =
                recvfrom(socket_fd_, buffer, sizeof(buffer), 0, nullptr, nullptr);
            if (received < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;
                if (errno == EINTR)
                    continue;
                ROS_WARN_THROTTLE(2.0,
                                  "Error receiving SIYI UDP packet: %s",
                                  std::strerror(errno));
                break;
            }

            uint8_t command_id = 0;
            std::vector<uint8_t> payload;
            if (!decodePacket(buffer,
                              static_cast<std::size_t>(received),
                              command_id,
                              payload))
                continue;

            if (command_id == 0x0D && payload.size() >= 6)
            {
                yaw_deg_ = static_cast<double>(readInt16LE(payload.data())) / 10.0;
                pitch_deg_ = static_cast<double>(readInt16LE(payload.data() + 2)) / 10.0;
                roll_deg_ = static_cast<double>(readInt16LE(payload.data() + 4)) / 10.0;
                last_attitude_time_ = now;
                has_attitude_ = true;
            }
        }
    }

    bool hasFreshAttitude(SteadyClock::time_point now) const
    {
        return has_attitude_ && (now - last_attitude_time_) < std::chrono::seconds(1);
    }

    double yaw_deg_ = 0.0;
    double pitch_deg_ = 0.0;
    double roll_deg_ = 0.0;

private:
    int socket_fd_ = -1;
    sockaddr_in server_addr_{};
    uint16_t sequence_ = 0;
    bool has_attitude_ = false;
    SteadyClock::time_point last_attitude_time_{};
    SteadyClock::time_point next_attitude_request_{};

    void closeSocket()
    {
        if (socket_fd_ >= 0)
        {
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
        has_attitude_ = false;
    }

    bool sendPacket(uint8_t command_id, const std::vector<uint8_t> &payload)
    {
        if (socket_fd_ < 0)
            return false;
        if (payload.size() > 255)
            return false;

        std::vector<uint8_t> packet;
        packet.reserve(10 + payload.size());
        packet.push_back(0x55);
        packet.push_back(0x66);
        packet.push_back(0x01); // Request/command packet.
        packet.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
        packet.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFF));
        packet.push_back(static_cast<uint8_t>(sequence_ & 0xFF));
        packet.push_back(static_cast<uint8_t>((sequence_ >> 8) & 0xFF));
        packet.push_back(command_id);
        packet.insert(packet.end(), payload.begin(), payload.end());
        ++sequence_;

        const uint16_t crc = siyiCrc16(packet.data(), packet.size());
        packet.push_back(static_cast<uint8_t>(crc & 0xFF));
        packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

        const ssize_t sent = sendto(socket_fd_,
                                    packet.data(),
                                    packet.size(),
                                    0,
                                    reinterpret_cast<const sockaddr *>(&server_addr_),
                                    sizeof(server_addr_));
        if (sent != static_cast<ssize_t>(packet.size()))
        {
            ROS_WARN_THROTTLE(2.0,
                              "Failed to send SIYI command 0x%02X: %s",
                              command_id,
                              std::strerror(errno));
            return false;
        }
        return true;
    }

    bool decodePacket(const uint8_t *buffer,
                      std::size_t size,
                      uint8_t &command_id,
                      std::vector<uint8_t> &payload) const
    {
        if (size < 10 || buffer[0] != 0x55 || buffer[1] != 0x66)
            return false;

        const std::size_t data_length = static_cast<std::size_t>(buffer[3]) |
                                        (static_cast<std::size_t>(buffer[4]) << 8);
        const std::size_t packet_length = 8 + data_length + 2;
        if (size < packet_length)
            return false;

        const uint16_t received_crc =
            static_cast<uint16_t>(buffer[packet_length - 2]) |
            (static_cast<uint16_t>(buffer[packet_length - 1]) << 8);
        if (siyiCrc16(buffer, packet_length - 2) != received_crc)
            return false;

        command_id = buffer[7];
        payload.assign(buffer + 8, buffer + 8 + data_length);
        return true;
    }
};

SiyiGimbalUdp g_gimbal;
void publishStatus(const std::string &status)
{
    if (status == g_last_status)
        return;
    g_last_status = status;

    std_msgs::String msg;
    msg.data = status;
    g_status_publisher.publish(msg);
}

void landingCallback(const std_msgs::Bool::ConstPtr &msg)
{
    g_landing_requested.store(msg->data);
}

void px4LandingStateCallback(const mavros_msgs::ExtendedStateConstPtr &msg)
{
    land_state = msg->landed_state;
}

void SIYI_ROS_SDK::pubLandPose(Eigen::Vector3d &pose)
{
    geometry_msgs::Point msg;
    msg.x = pose.x();
    msg.y = pose.y();
    msg.z = pose.z();
    landing_pose_pub.publish(msg);
}

// This method is expected to be called periodically by the existing application.
void SIYI_ROS_SDK::process(void)
{
    const auto now = SteadyClock::now();
    g_gimbal.updateAttitude(now);

    bool landing_requested = g_landing_requested.load();

    switch (g_state)
    {
    case IDLE:
    {
        if (landing_requested)
        {
            g_gimbal.setMode(SiyiGimbalUdp::FPV_MODE);
	    ros::Duration(0.1).sleep();   // 等待 100ms
            // camera center-and-down
            if (g_gimbal.sendOneKeyPosition(2))
            {
                g_state = MOVING_DOWN;
                publishStatus("MOVING_DOWN");
            }
        }
        else
            publishStatus("IDLE");

        break;
    }
    // 云台往下转
    case MOVING_DOWN:
    {
        if (g_gimbal.hasFreshAttitude(now) && g_gimbal.pitch_deg_ <= -90 && std::abs(g_gimbal.yaw_deg_) <= 0.0001)
        {
            g_state = MOVE_LANDING_POS;
            publishStatus("MOVE_LANDING_POS");
            ROS_INFO("Let drone go to aprilTag center");
        }

        break;
    }

    case MOVE_LANDING_POS:
    {
        Eigen::Vector2d e = p.head<2>() - land_target.head<2>();
        double e_p = e.norm();
        // Current body yaw in world frame
        double yaw_current = atan2(R_b2w(1, 0), R_b2w(0, 0));
        double des_yaw = land_target.z();
        double yaw_error =
            atan2(sin(des_yaw - yaw_current), cos(des_yaw - yaw_current));
        double e_yaw = std::abs(yaw_error);
        constexpr double yaw_tol = 1.0 * M_PI / 180.0;
        ROS_WARN("e_x: %.2f, e_y: %.2f, yaw: %.2f",
                 e.x(),
                 e.y(),
                 e_yaw / yaw_tol);
        if (e_p < 0.1 && e_yaw < yaw_tol && v.norm() < 0.1)
        {
            g_state = LANDING;
            is_updated = false;
            publishStatus("LANDING");
            ROS_INFO("Drone is at the place. Ready to land!");
        }
        pubLandPose(land_target);
        break;
    }

    case LANDING:
        pubLandPose(land_pose);
        if (land_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND ||
            (v.norm() < 0.01 && p.z() < 0.05))
        {
            g_state = IDLE;
            publishStatus("IDLE");

            landing_requested = false;
            g_landing_requested = false;
            g_gimbal.sendOneKeyPosition(1);
            ROS_INFO("Landing stopped; one-key gimbal center commanded.");
        }
        break;
    }
}

void SIYI_ROS_SDK::tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (g_state >= MOVE_LANDING_POS && rev_odom && msg->detections.size() != 0)
    {
        Eigen::Quaterniond q_t2c;
        const auto &det = msg->detections[0];
        q_t2c.w() = det.pose.pose.pose.orientation.w;
        q_t2c.x() = det.pose.pose.pose.orientation.x;
        q_t2c.y() = det.pose.pose.pose.orientation.y;
        q_t2c.z() = det.pose.pose.pose.orientation.z;
        Eigen::Vector3d p_t2c;
        p_t2c.x() = det.pose.pose.pose.position.x;
        p_t2c.y() = det.pose.pose.pose.position.y;
        p_t2c.z() = det.pose.pose.pose.position.z;

        Eigen::Matrix3d R_t2c = q_t2c.toRotationMatrix();

        // Virtual target point in Tag frame
        Eigen::Vector3d p_q_t(0.0, 0.08, 0.0); // 8cm
	const Eigen::Vector3d t_b2c(0.08, 0.0, 0.0); // 8cm
        Eigen::Vector3d p_q_c = p_t2c + R_t2c * p_q_t;
        Eigen::Vector3d target = (p + R_b2w * R_c2b * p_q_c);

        Eigen::Matrix3d R_t2w = R_b2w * R_c2b * R_t2c;
        Eigen::Vector3d tag_w_y = R_t2w.col(1);
        double yaw_target = atan2(tag_w_y.y(), tag_w_y.x());

        land_pose.x() = target.x();
        land_pose.y() = target.y();
        land_pose.z() = yaw_target; // axis z serve as yaw target
        if (!is_updated && g_state == MOVE_LANDING_POS)
        {
            is_updated = true;
            land_target = land_pose;
        }
    }
}

void SIYI_ROS_SDK::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    rev_odom = true;
    p.x() = msg->pose.pose.position.x;
    p.y() = msg->pose.pose.position.y;
    p.z() = msg->pose.pose.position.z;

    v.x() = msg->twist.twist.linear.x;
    v.y() = msg->twist.twist.linear.y;
    v.z() = msg->twist.twist.linear.z;

    Eigen::Quaterniond q;
    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    R_b2w = q.toRotationMatrix();
}

SIYI_ROS_SDK::SIYI_ROS_SDK(void)
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int gimbal_port;
    std::string gimbal_ip;
    std::string landing_topic;
    std::string status_topic;
    pnh.param("gimbal_ip", gimbal_ip, std::string("192.168.144.25"));
    pnh.param("gimbal_port", gimbal_port, 37260);
    pnh.param("landing_topic", landing_topic, std::string("/landing_camera/set_landing"));
    pnh.param("status_topic", status_topic, std::string("/landing_camera/status"));

    tag_sub = pnh.subscribe("tag_topic", 10, &SIYI_ROS_SDK::tagCallback, this);
    odom_sub = pnh.subscribe("odom", 10, &SIYI_ROS_SDK::odomCallback, this);
    g_landing_subscriber = nh.subscribe(landing_topic, 1, landingCallback);

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                 10,
                                                 px4LandingStateCallback);

    g_status_publisher = nh.advertise<std_msgs::String>(status_topic, 1, true);
    landing_pose_pub =
        nh.advertise<geometry_msgs::Point>("/landing_camera/landing_pose", 1, true);

    /* 下视 camera */
    R_c2b.col(0) << 0, -1, 0;
    R_c2b.col(1) << -1, 0, 0;
    R_c2b.col(2) << 0, 0, -1;
    p.setZero();
    R_b2w.setIdentity();

    land_pose.setZero();
    land_target.setZero();

    if (!g_gimbal.initialize(gimbal_ip, gimbal_port))
    {
        ROS_ERROR("SIYI gimbal control socket initialization failed. Gimbal sequencing "
                  "will not work.");
    }
}
