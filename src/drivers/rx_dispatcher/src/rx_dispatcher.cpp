#include <memory>
#include <libserial/SerialPort.h>
#include "rx_dispatcher.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"






int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto rx_node = std::make_shared<RxDispatcher<std_msgs::msg::UInt8MultiArray, example_interfaces::msg::Int8MultiArray>>();
    rclcpp::spin(rx_node);
    rclcpp::shutdown();
    return 0;
}
