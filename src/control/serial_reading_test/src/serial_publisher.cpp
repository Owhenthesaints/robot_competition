#include <iostream>
#include <boost/asio.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
using namespace std::chrono_literals;


template<typename MessageType>
class SerialSender : public rclcpp::Node {
public:
    SerialSender(const char * arduino_input = "/dev/ttyACM0"): Node("serial_sender"), io(), serial(io, arduino_input){
        //setup timer
        timer_ = this->create_wall_timer(25ms, std::bind(&SerialSender::timer_callback, this));
        //setup publisher
        publisher_ = this->create_publisher<MessageType>("/local_avoidance/instructions", 10);
        //setup serial
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
    }
private:
    void timer_callback(){
        char c;
        read(serial, boost::asio::buffer(&c, 1));
        RCLCPP_INFO(this->get_logger(), "recieved '%c'", c);
    }

    boost::asio::io_service io;
    boost::asio::serial_port serial;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Publisher<MessageType>> publisher_;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SerialSender<std_msgs::msg::Int16>>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();
}