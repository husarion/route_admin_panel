#include <MapAsImageProvider.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAsImageProvider>());
  rclcpp::shutdown();
  return 0;
}
