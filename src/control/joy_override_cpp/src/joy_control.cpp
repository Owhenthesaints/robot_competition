#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

typedef sensor_msgs::msg::Joy joyIncomming;

template<typename IncommingMessage>
class JoyConverter : public rclcpp::Node {
public:
    JoyConverter(): Node("joy_converter"){
        publisher_ = this->create_publisher<publishType>("motor_updates/direction", 10);
        timer_ = this -> create_wall_timer( 500ms, std::bind(&JoyConverter::timer_callback, this));
        subscription_ = this->create_subscription<IncommingMessage>("joy", 10, std::bind(&JoyConverter::topic_callback, this, _1));
    }

private:
    typedef example_interfaces::msg::Int8MultiArray publishType;

    void topic_callback(const IncommingMessage & joy_message) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", joy_message.axes[0]);
    }

    void timer_callback() {
        auto message = publishType();
        message.data = {3, 4};
        publisher_ -> publish(message);
    }

    std::shared_ptr<rclcpp::TimerBase> timer_;
    std::shared_ptr<rclcpp::Publisher<publishType>> publisher_;
    std::shared_ptr<rclcpp::Subscription<IncommingMessage>> subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyConverter<joyIncomming>>());
    rclcpp::shutdown();
    return 0;
}
