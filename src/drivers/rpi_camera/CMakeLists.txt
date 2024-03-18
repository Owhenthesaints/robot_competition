cmake_minimum_required(VERSION 3.8)
project(rpi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(opencv REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executables(camera_tranciever src/camera_node.cpp)
ament_target_dependencies(camera_tranciever rclcpp sensor_msgs)
target_link_libraries(camera_tranciever ${OpenCV_INCLUDE_DIRS})

install (TARGETS
        rotor_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${dependencies})
ament_package()

ament_package()
