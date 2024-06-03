cmake_minimum_required(VERSION 3.8)
project(main_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(Armadillo REQUIRED)

add_executable(main_control_cpp src/main_control_cpp.cpp src/main_controller.cpp)
ament_target_dependencies(main_control_cpp std_msgs rclcpp sensor_msgs example_interfaces vision_msgs)
# Link the Armadillo library
target_link_libraries(main_control_cpp ${ARMADILLO_LIBRARIES})

# Include the Armadillo directories
target_include_directories(main_control_cpp PRIVATE ${ARMADILLO_INCLUDE_DIRS})
install(TARGETS
    main_control_cpp
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
