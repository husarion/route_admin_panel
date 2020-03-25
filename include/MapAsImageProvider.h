#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "rclcpp/rclcpp.hpp"

#ifndef MAP_AS_IMAGE_PROVIDER_H
#define MAP_AS_IMAGE_PROVIDER_H

#define INITIAL_MAP_SIZE_X 768
#define INITIAL_MAP_SIZE_Y 768
#define INITIAL_FULL_MAP_DELAY 1

class MapAsImageProvider : public rclcpp::Node
{
private:
  sensor_msgs::msg::Image tmp_image_data;
  image_transport::ImageTransport* image_transport_;
  image_transport::Publisher image_transport_publisher_full_;
  nav_msgs::msg::MapMetaData map_meta_data;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr map_metadata_publisher_;
  nav_msgs::msg::OccupancyGrid currentMap;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  cv_bridge::CvImage cv_img_full_;

  rclcpp::Time* lastMapUpdate;
  rclcpp::Duration* fullMapDelay;

  void mapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

  bool metadata;

public:
  MapAsImageProvider();
  ~MapAsImageProvider();
  void publishFullMap(bool force = false);
};

#endif