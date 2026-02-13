cmake_minimum_required(VERSION 3.8)
project(assignment2_rt)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/SetThreshold.srv"
  "srv/GetAvgVel.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
