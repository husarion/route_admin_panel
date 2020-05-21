#include <math.h>
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#ifndef AMCL_ESTIMATE_H
#define AMCL_ESTIMATE_H

class AmclEstimate
{
public:
    AmclEstimate(ros::NodeHandle nh);
    ~AmclEstimate();
    bool update_estimate();

private:
    ros::NodeHandle node_handle;
    std::string global_localization_service;
    std::string nomotion_update_service;
    std::string map_topic;
    std::string scan_topic;
    bool map_received;
    double scan_x;
    double scan_y;
    double scan_theta;
    double range_threshold;
    double estimate_minimum_score;
    double estimate_current_score;
    u_int8_t nomotion_updates;
    int nomotion_max_retries;
    ros::Time last_loc_update;
    std_srvs::Empty localization_req_res;
    nav_msgs::OccupancyGrid current_map;
    sensor_msgs::LaserScan current_scan;
    ros::ServiceClient _localization_client;
    ros::ServiceClient _nomotion_update_client;
    ros::Subscriber _map_subscriber;
    ros::Subscriber _scan_subscriber;
    tf::StampedTransform scan_transform;
    tf::TransformListener *listener;
    void map_callback(const nav_msgs::OccupancyGridConstPtr &map);
    void scan_callback(const sensor_msgs::LaserScanConstPtr &scan);
    bool call_global_localization();
    bool call_nomotion_update();
    double calculate_match();
    bool is_obstacle_at_point(double x, double y);
    bool is_obstacle_at_range(double x, double y, double range, double *dist_found);
    bool is_point_within_map_range(double x, double y);
};
#endif