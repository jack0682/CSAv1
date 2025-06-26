cmake_minimum_required(VERSION 3.5)
project(csa_slam_interface)

# C++ 표준
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pangolin 빌드 경로
set(PANGOLIN_LIB_DIR /home/jack/Pangolin/build)
link_directories(${PANGOLIN_LIB_DIR})
include_directories(${PANGOLIN_LIB_DIR}/include)

# ORB-SLAM2 include 및 lib 경로
include_directories(
  include
  /home/jack/ORB_SLAM2
  /home/jack/ORB_SLAM2/include
  /home/jack/ORB_SLAM2/Thirdparty/DBoW2
  /home/jack/ORB_SLAM2/Thirdparty/g2o
  /home/jack/ORB_SLAM2/Thirdparty/Sophus
  /usr/include/eigen3
)
link_directories(/home/jack/ORB_SLAM2/lib)

# ROS2 및 기타 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

# 실행 파일
add_executable(slam_pose_node src/slam_pose_node.cpp)

# ROS2 의존성
ament_target_dependencies(slam_pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

# 라이브러리 링크
target_link_libraries(slam_pose_node
  ORB_SLAM2
  pango_core
  pango_display
  pango_windowing
  pango_geometry
  pango_vars
  pango_opengl
  pango_glgeometry
  pango_image
  pango_plot
  pango_scene
  GL      # OpenGL 링커 누락 방지
  ${GLEW_LIBRARIES}
  ${OpenCV_LIBS}
)

# 설치 설정
install(TARGETS slam_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
