cmake_minimum_required(VERSION 3.8)
project(video_stream_opencv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/video_stream.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "video_stream_opencv::VideoStreamNode"
  EXECUTABLE video_stream
)

install(TARGETS
  ${PROJECT_NAME}
  video_stream
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_video_resource.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()