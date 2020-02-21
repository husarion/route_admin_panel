cmake_minimum_required(VERSION 3.5)
project(route_admin_panel)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)

####################
## Add executable ##
####################

include_directories(
  include
)

set(executable_name map_to_img_node)
add_executable(${executable_name}
  src/map_to_img.cpp 
  src/MapAsImageProvider.cpp
)

ament_target_dependencies(${executable_name}
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "tf2"
  "tf2_msgs"
  "tf2_ros"
  "image_transport"
  "cv_bridge"
  "nav_msgs"
)

#############
## Install ##
#############

install(TARGETS ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config images include launch nodejs scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

ament_package()
