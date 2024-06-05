#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <chrono>

using namespace std::chrono_literals;
#define WALL_TIMER_LOOPBACK 100ms

template<typename PublishType>
class DemoProximitySensor : public rclcpp::Node {
public:
    DemoProximitySensor() : rclcpp::Node("demo_prox_sensor") {
        publisher_ = this->create_publisher<PublishType>("rx/distance/value", 10);
        timer_ = this->create_wall_timer(WALL_TIMER_LOOPBACK, [this](){ timer_callback(); });
    }

private:
    void timer_callback() {
        PublishType message;
        message.data.push_back(25);
        message.data.push_back(100);
        message.data.push_back(100);
        message.data.push_back(100);
        message.data.push_back(100);
        RCLCPP_INFO(this->get_logger(), "publishing distance");
        publisher_->publish(message);
    }
    std::shared_ptr<rclcpp::Publisher<PublishType>> publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DemoProximitySensor<std_msgs::msg::UInt8MultiArray>>();
    rclcpp::spin(node);
    return 0;
}
