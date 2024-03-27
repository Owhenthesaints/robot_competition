cmake_minimum_required(VERSION 3.8)
project(rpi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(camera_tranciever src/camera_node.cpp)
ament_target_dependencies(camera_tranciever rclcpp sensor_msgs  cv_bridge std_msgs)
target_link_libraries(camera_tranciever ${OpenCV_LIBS})

install (TARGETS
        camera_tranciever
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

