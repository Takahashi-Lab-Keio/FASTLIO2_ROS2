cmake_minimum_required(VERSION 3.8)
project(fastlio2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationState.msg"
  "srv/SaveMaps.srv"
  "srv/Relocalize.srv"
  "srv/IsValid.srv"
  "srv/RefineMap.srv"
  "srv/SavePoses.srv"
  DEPENDENCIES std_msgs geometry_msgs
)



# Focused regression tests live in fastlio2; upstream-wide style normalization
# is intentionally out of scope for this PoC fork.

ament_package()
