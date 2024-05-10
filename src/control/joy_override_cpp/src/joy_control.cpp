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

#define MAX_SPEED 100

typedef sensor_msgs::msg::Joy joyIncomming;

template<typename IncommingMessage>
class JoyConverter : public rclcpp::Node {
public:
    JoyConverter(): Node("joy_converter"), axes_array_(new float[2]){
        axes_array_[0] = 0;
        axes_array_[1] = 1;
        publisher_ = this->create_publisher<publishType>("motor_updates/direction", 10);
        timer_ = this -> create_wall_timer( 75ms, std::bind(&JoyConverter::timer_callback, this));
        subscription_ = this->create_subscription<IncommingMessage>("joy", 10, std::bind(&JoyConverter::topic_callback, this, _1));
    }

private:
    typedef example_interfaces::msg::Int8MultiArray publishType;

    void topic_callback(const IncommingMessage & joy_message) {
        axes_array_[0] = joy_message.axes[0];
        axes_array_[1] = joy_message.axes[1];
    }

    void timer_callback()
    {
        publishType message;
        // setting the motor values
        int left = static_cast<int>((axes_array_[1] - axes_array_[0]) * MAX_SPEED);
        int right = static_cast<int>((axes_array_[1] + axes_array_[0]) * MAX_SPEED);
        // capping motor speeds to 100%
        int8_t left_motor = static_cast<int8_t>(std::max(std::min(left, MAX_SPEED), -MAX_SPEED));
        int8_t right_motor = static_cast<int8_t>(std::max(std::min(right, MAX_SPEED), -MAX_SPEED));
        // publish
        message.data = {left_motor, right_motor};
        publisher_->publish(message);
    }

    std::shared_ptr<rclcpp::TimerBase> timer_;
    std::shared_ptr<rclcpp::Publisher<publishType>> publisher_;
    std::shared_ptr<rclcpp::Subscription<IncommingMessage>> subscription_;
    // axis[0] left pos right neg axis[1] front pos back neg
    std::unique_ptr<float[]> axes_array_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyConverter<joyIncomming>>());
    rclcpp::shutdown();
    return 0;
}
