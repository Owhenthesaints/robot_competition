cmake_minimum_required(VERSION 3.5)
project(rotor_control)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
set(DEBUG 1)
set(VECTOR_INSTRUCTIONS 1)



add_executable(rotor_control src/rotor_control.cpp)
ament_target_dependencies(rotor_control rclcpp example_interfaces)
target_link_libraries(rotor_control wiringPi)

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
