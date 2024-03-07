cmake_minimum_required(VERSION 3.8)
project(on_off_send_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(alternator src/alternator.cpp)
ament_target_dependencies(alternator rclcpp std_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(TARGETS alternator DESTINATION lib/${PROJECT_NAME})

ament_package()
