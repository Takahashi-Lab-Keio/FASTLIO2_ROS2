cmake_minimum_required(VERSION 3.8)
project(fastlio2_localizer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fastlio2_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)



include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


set(SRC_LIST src/localizers/commons.cpp
             src/localizers/icp_localizer.cpp)

add_library(fastlio2_localizer_core ${SRC_LIST})
target_include_directories(fastlio2_localizer_core PUBLIC src ${PCL_INCLUDE_DIRS})
target_link_libraries(fastlio2_localizer_core ${PCL_LIBRARIES})

add_executable(localizer_node src/localizer_node.cpp)
ament_target_dependencies(localizer_node rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs fastlio2_interfaces)
target_link_libraries(localizer_node fastlio2_localizer_core yaml-cpp)


install(TARGETS localizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_icp_localizer test/test_icp_localizer.cpp)
  target_link_libraries(test_icp_localizer fastlio2_localizer_core)

  ament_add_gtest(test_runtime_overrides test/test_runtime_overrides.cpp)
  target_include_directories(test_runtime_overrides PRIVATE src)

  ament_add_pytest_test(
    test_localization_launch
    test/test_localization_launch.py
  )
endif()

ament_package()
