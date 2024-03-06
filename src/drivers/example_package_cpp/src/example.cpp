#include "rclcpp/rclcpp.hpp"
#include <wiringPi.h>

int main(int argc, char* argv[]) {
    wiringPiSetup();

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("simple_node");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
