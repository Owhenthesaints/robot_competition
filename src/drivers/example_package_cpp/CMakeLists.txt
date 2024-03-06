cmake_minimum_required(VERSION 3.5)
project(example_package_cpp)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


set(dependencies rclcpp)

add_executable(example src/example.cpp)
ament_target_dependencies(example ${dependencies})
target_link_libraries(example wiringPi)

install (TARGETS
         example
         ARCHIVE DESTINATION lib
         LIBRARY DESTINATION lib
         RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${dependencies})
ament_package()
