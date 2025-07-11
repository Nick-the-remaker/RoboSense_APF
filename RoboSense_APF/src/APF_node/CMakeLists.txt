cmake_minimum_required(VERSION 3.0.2)
project(APF_node)

set(CMAKE_CXX_FLAGS "-std=c++14")

# set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  message_generation 
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  pcl_ros
)

find_package(OpenCV 4 REQUIRED )

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(APF_node src/APF_node.cpp)


target_link_libraries(APF_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  pcl_visualization
  pcl_common
  pcl_io
  pcl_features
  pcl_visualization
  pcl_kdtree
  pcl_search
)



