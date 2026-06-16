cmake_minimum_required(VERSION 3.10)
project(adaptive_fast_lio2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(Threads REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(adaptive_fastlio_mapping
  include/ikd-Tree/ikd_Tree.cpp
  src/adaptive_laserMapping.cpp
  src/adaptive_preprocess.cpp
  src/adaptive_imu_process.cpp
  src/adaptive_map_manager.cpp
  src/adaptive_runtime_logger.cpp
)

ament_target_dependencies(adaptive_fastlio_mapping
  rclcpp
  sensor_msgs
  livox_ros_driver2
  pcl_conversions
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(adaptive_fastlio_mapping
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  Eigen3::Eigen
  Threads::Threads
)

install(TARGETS adaptive_fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
