cmake_minimum_required(VERSION 3.8)
project(fastlio2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenMP QUIET)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

set(SRC_LIST src/map_builder/commons.cpp
             src/map_builder/ieskf.cpp
             src/map_builder/imu_processor.cpp
             src/map_builder/ikd_Tree.cpp
             src/map_builder/lidar_processor.cpp
             src/map_builder/map_builder.cpp
             src/utils.cpp)

add_library(fastlio2_core ${SRC_LIST})
target_include_directories(fastlio2_core PUBLIC src ${PCL_INCLUDE_DIRS})
ament_target_dependencies(fastlio2_core std_msgs sensor_msgs pcl_conversions)
target_link_libraries(fastlio2_core ${PCL_LIBRARIES})

add_executable(lio_node src/lio_node.cpp)
ament_target_dependencies(lio_node rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs pcl_conversions geometry_msgs)
target_link_libraries(lio_node
  fastlio2_core
  yaml-cpp
)


install(TARGETS lio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # This fork runs focused regressions below. Normalizing all inherited upstream
  # sources with ament_auto style tools is intentionally out of scope.

  ament_add_gtest(test_pointcloud2_conversion test/test_pointcloud2_conversion.cpp)
  target_link_libraries(test_pointcloud2_conversion fastlio2_core)
  ament_target_dependencies(test_pointcloud2_conversion sensor_msgs)

  ament_add_gtest(test_lidar_jacobian test/test_lidar_jacobian.cpp)
  target_link_libraries(test_lidar_jacobian fastlio2_core)

  ament_add_gtest(test_output_transforms test/test_output_transforms.cpp)
  target_link_libraries(test_output_transforms fastlio2_core)

  ament_add_gtest(test_so3_math test/test_so3_math.cpp)
  target_link_libraries(test_so3_math fastlio2_core)

  ament_add_gtest(test_runtime_overrides test/test_runtime_overrides.cpp)
  target_include_directories(test_runtime_overrides PRIVATE src)

  ament_add_pytest_test(test_lio_launch test/test_lio_launch.py)
endif()

ament_package()
