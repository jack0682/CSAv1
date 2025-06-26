cmake_minimum_required(VERSION 3.8)
project(csa_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
