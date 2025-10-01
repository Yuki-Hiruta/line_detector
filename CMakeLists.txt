cmake_minimum_required(VERSION 3.5)
project(proto_line_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(line_detector_node src/line_detector.cpp)
ament_target_dependencies(line_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS
  line_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
