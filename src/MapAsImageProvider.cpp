#include "MapAsImageProvider.h"

MapAsImageProvider::MapAsImageProvider() : Node("map_to_img_node")
{
  metadata = false;
  this->declare_parameter("publish_map_metadata");
  if (this->get_parameter("publish_map_metadata", metadata))
  {
    RCLCPP_INFO(this->get_logger(), "publish_map_metadata declared");
    if (metadata)
    {
      RCLCPP_INFO(this->get_logger(), "Will publish metadata");
    }
  }
  
  image_transport_ = new image_transport::ImageTransport(static_cast<rclcpp::Node::SharedPtr>(this));
  lastMapUpdate = new rclcpp::Time();
  image_transport_publisher_full_ = image_transport_->advertise("/map_image/full", 1, true);
  cv_img_full_.header.frame_id = "map_image";
  cv_img_full_.encoding = sensor_msgs::image_encodings::MONO8;
  cv_img_full_.image = cv::Mat(INITIAL_MAP_SIZE_Y, INITIAL_MAP_SIZE_X, CV_8U, 127);
  *lastMapUpdate = this->now();
  fullMapDelay = new rclcpp::Duration(INITIAL_FULL_MAP_DELAY);

  if (metadata)
  {
    map_metadata_publisher_ = this->create_publisher<nav_msgs::msg::MapMetaData>("/map_metadata", 1);
  }

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1, std::bind(&MapAsImageProvider::mapUpdate, this, std::placeholders::_1));

  currentMap.info.resolution = 1;
}

MapAsImageProvider::~MapAsImageProvider()
{
  delete image_transport_;
}

void MapAsImageProvider::publishFullMap(bool force)
{
  if (force)
  {
    cv_img_full_.header.stamp = this->now();
    tmp_image_data = *cv_img_full_.toImageMsg();
    image_transport_publisher_full_.publish(tmp_image_data);
    *lastMapUpdate = this->now();
  }
  else if (*lastMapUpdate + *fullMapDelay < this->now())
  {
    cv_img_full_.header.stamp = this->now();
    tmp_image_data = *cv_img_full_.toImageMsg();
    image_transport_publisher_full_.publish(tmp_image_data);
    *lastMapUpdate = this->now();
  }
  if (metadata)
  {
    map_meta_data = currentMap.info;
    map_metadata_publisher_->publish(map_meta_data);
  }
}

void MapAsImageProvider::mapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  if ((map->info.width < 3) || (map->info.height < 3))
  {
    RCLCPP_WARN(this->get_logger(), "Map size is only x: %d,  y: %d . Not running map to image conversion",
                map->info.width, map->info.height);
    return;
  }
  currentMap.header = map->header;
  currentMap.info = map->info;
  currentMap.data = map->data;

  // resize cv image if it doesn't have the same dimensions as the map
  if (((uint)cv_img_full_.image.rows != map->info.height) || ((uint)cv_img_full_.image.cols != map->info.width))
  {
    cv_img_full_.image = cv::Mat(map->info.height, map->info.width, CV_8U);
  }
  // We have to flip around the y axis, y for image starts at the top and y for map at the bottom
  int size_y_rev = cv_img_full_.image.rows - 1;
  for (int y = size_y_rev; y >= 0; --y)
  {
    int idx_map_y = cv_img_full_.image.cols * y;
    int idx_img_y = cv_img_full_.image.cols * (cv_img_full_.image.rows - y - 1);

    for (int x = 0; x < cv_img_full_.image.cols; ++x)
    {
      // int map_idx = idx_map_y + x;
      int idx = idx_img_y + x;
      switch (currentMap.data[idx_map_y + x])
      {
        case -1:
          cv_img_full_.image.data[idx] = 127;
          break;
        case 0:
          cv_img_full_.image.data[idx] = 255;
          break;
        case 100:
          cv_img_full_.image.data[idx] = 0;
          break;
      }
    }
  }
  publishFullMap(true);
  return;
}
