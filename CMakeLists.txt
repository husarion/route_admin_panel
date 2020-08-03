cmake_minimum_required(VERSION 2.8.3)
project(route_admin_panel)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp tf
)

find_package(catkin REQUIRED)
catkin_package( )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(amcl_estimate_node src/amcl_estimate_node.cpp src/AmclEstimate.cpp)

target_link_libraries(amcl_estimate_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch nodejs
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(TARGETS amcl_estimate_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
