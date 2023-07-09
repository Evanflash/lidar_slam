cmake_minimum_required(VERSION 3.8)
project(slam_learn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(other_msgs REQUIRED)
find_package(Ceres REQUIRED)

include_directories(include/slam_learn)
ament_export_include_directories(include)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  other_msgs
  pcl_conversions
)

add_executable(data_process_node src/data_process.cpp)
target_link_libraries(data_process_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(data_process_node ${dependencies} Eigen3)

add_executable(front_end_node src/front_end.cpp)
target_link_libraries(front_end_node
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)
ament_target_dependencies(front_end_node ${dependencies} Eigen3)

# test
add_executable(data_process_test test/data_process_test.cpp)
target_link_libraries(data_process_test
  ${PCL_LIBRARIES}
)
ament_target_dependencies(data_process_test ${dependencies})


install(TARGETS
  data_process_node
  front_end_node

  data_process_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
