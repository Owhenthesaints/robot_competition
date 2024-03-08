#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include <wiringPi.h>
#include <memory>

#define ALTERNATING_PIN 24

using std::placeholders::_1;

class pinWriter : public rclcpp::Node {

public:
    pinWriter(): Node("pin_writter")
    {
        subscription_ = this-> create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&pinWriter::topic_callback, this, _1));
    }
private:
    void topic_callback(const std_msgs::msg::String & msg) const {
        std::string s = std::string(msg.data.c_str());
        if(s.back()=='f'){
            digitalWrite(ALTERNATING_PIN, HIGH);
            RCLCPP_INFO(this->get_logger(), "turning on: '%s'", msg.data.c_str());
        }else{
            digitalWrite(ALTERNATING_PIN, LOW);
            RCLCPP_INFO(this->get_logger(), "turning off: '%s'", msg.data.c_str());
        }

        

    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};

int main(int argc, char* argv[]) {
    wiringPiSetup();
    pinMode(ALTERNATING_PIN, OUTPUT);

    rclcpp::init(argc, argv);


    rclcpp::spin(std::make_shared<pinWriter>());

    rclcpp::shutdown();

    return 0;
}
