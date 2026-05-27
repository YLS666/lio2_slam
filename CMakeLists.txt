cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(lio2_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (
  CMAKE_BUILD_TYPE
  "Debug"
  CACHE STRING "Choose Release or Debug" FORCE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options (-fdiagnostics-color=always)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(TBB REQUIRED)

set(
    ament_dependencies
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_srvs
    pcl_conversions
    rosbag2_cpp
)

# 第三方库
include_directories (${PROJECT_SOURCE_DIR}/thirdparty/sophus)

find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})

find_package (yaml-cpp REQUIRED)
include_directories (${yaml-cpp_INCLUDE_DIRS})

# find_package (G2O REQUIRED)
# include_directories (${G2O_INCLUDE_DIRS})

find_package (Pangolin REQUIRED)
include_directories (${Pangolin_INCLUDE_DIRS})

set (
    third_party_libs
    ${PCL_LIBRARIES}
    yaml-cpp
    tbb
)

include_directories (${PROJECT_SOURCE_DIR}/src)

find_package(OpenMP QUIET)

add_executable(run_mapping 
 src/app/run_lio_mapper.cpp
 src/cloud_utils/cloud_processor.cpp 
 src/imu_utils/imu_processor.cpp 
 src/registration/scan_to_map.cpp 
 src/ros_bridge/bag_io.cpp
 src/sync/time_sync.cpp
 )

target_link_libraries(run_mapping 
    ${third_party_libs}
)

ament_target_dependencies (run_mapping ${ament_dependencies})

install(TARGETS
  run_mapping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
