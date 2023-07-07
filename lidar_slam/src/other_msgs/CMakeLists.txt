cmake_minimum_required(VERSION 3.8)
project(other_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point.msg"
  "msg/AllCloud.msg"
  "msg/SegCloud.msg"
  DEPENDENCIES std_msgs
)

ament_package()
