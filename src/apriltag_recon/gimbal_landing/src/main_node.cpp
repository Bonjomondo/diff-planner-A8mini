#include "gimbal_landing.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_landing");
    SIYI_ROS_SDK siyi_ros_sdk;

    ros::Rate r(30);

    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        siyi_ros_sdk.process();
    }

    return 0;
}
