#include <ros/ros.h>
#include <AmclEstimate.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amcl_estimate_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(1);
    ROS_INFO("Init AmclEstimate object");
    AmclEstimate amcl_estimate(nh);
    return 0;
}