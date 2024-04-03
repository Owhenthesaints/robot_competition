cmake_minimum_required(VERSION 3.5)
project(rotor_control)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
set(DEBUG)
set(VECTOR_INSTRUCTIONS 1)

# uncoment if you want to send the value of 25 on both wheels
# add_compile_definitions(DEBUG=0)
add_compile_definitions(VECTOR_INSTRUCTIONS)


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
