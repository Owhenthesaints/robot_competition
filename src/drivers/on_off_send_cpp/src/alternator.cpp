#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AlternatorNode: public rclcpp::Node
{
    public:
    AlternatorNode(): Node("alternator"), on(false)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this-> create_wall_timer(1000ms, std::bind(&AlternatorNode::timer_callback, this));
    }
    private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String(); 
        message.data = "Publishing " + std::string(on? "f" : "t");
        on = !on;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    bool on;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlternatorNode>());
    rclcpp::shutdown();
}
