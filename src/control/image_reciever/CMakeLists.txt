cmake_minimum_required(VERSION 3.5)
project(image_reciever)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(image_reciever src/image_reciever.cpp)
ament_target_dependencies(image_reciever rclcpp sensor_msgs cv_bridge)
target_link_libraries(image_reciever ${OpenCV_LIBS})

install(TARGETS
  image_reciever
  DESTINATION lib/${PROJECT_NAME})

ament_package()