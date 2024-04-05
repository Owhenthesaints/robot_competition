#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int8.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <string>
#include <boost/asio.hpp>

// conversion constant to convert from 100 to 1024
#define CONVERSION_CONSTANT 10.23
// says we work in percentage (from axes 0-1 into percentage)
#define MAX_SPEED_CST 100
#define THRESHOLD 10
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))


// change communication type between two classes
typedef example_interfaces::msg::Int8 comType;
// change vector arrival type
typedef example_interfaces::msg::Int8MultiArray arrivalType;

using namespace std::chrono_literals;
using std::placeholders::_1;

/* 
 * @brief This is a higher level class that will handle the controls of the motors 
 * 
 * This class will hopefully offer a very flexible of inputting motor commands. The intended way for now will be 2d vectors transformed into
 * motor commands but through preprocessor directives we leave the door open to other ways
 * */
template<typename SubscriptionType>
class MainControlRotor : public rclcpp::Node
{
public:
    MainControlRotor(const char * portName="/dev/ttyACM0", int num_children = 2) : Node("main_motor_control"), io_(), port_(io_)
    {
        // setup port
        port_.open(portName);
        port_.set_option(boost::asio::serial_port_base::baud_rate(9600));
        // message of DEBUG severity
        RCLCPP_DEBUG(this-> get_logger(), "starting init MainControlRotor");
        //setup timer
        timer_ = this-> create_wall_timer(50ms, std::bind(&MainControlRotor::timer_callback, this));
        wheel_array_ = new int8_t[num_children];
        //define two publishers for different topics
        RCLCPP_DEBUG(this-> get_logger(), "starting vector instruction implementation init");
        wheel_array_[0] = 0;
        wheel_array_[1] = 0;
        subscriber_ = this-> create_subscription<SubscriptionType>("motor_updates/direction", 10, std::bind(&MainControlRotor::rotor_values_update, this, _1));
    }
private:

    void rotor_values_update(const SubscriptionType & msg){
        RCLCPP_INFO(this-> get_logger(), "updating motor values with '%d' and '%d'", msg.data[0], msg.data[1]);
        wheel_array_[0] = msg.data[0];
        wheel_array_[1] = msg.data[1];
    }

    void timer_callback(){
        RCLCPP_DEBUG(this->get_logger(), "entering timer callback");
        //
        std::string speed_left = std::string(1, static_cast<char>(MIN(MAX(wheel_array_[0], - MAX_SPEED_CST), MAX_SPEED_CST)));
        std::string speed_right = std::string(1, static_cast<char>(MIN(MAX(wheel_array_[1], - MAX_SPEED_CST), MAX_SPEED_CST)));
        boost::asio::write(port_, boost::asio::buffer(speed_left));
        boost::asio::write(port_, boost::asio::buffer(speed_right));
        boost::asio::write(port_, boost::asio::buffer(";"));
    }
    rclcpp::TimerBase::SharedPtr timer_;
    // wheel_array_[1] up down, joystick_array_[2] left right
    int8_t * wheel_array_;
    //if the class is built with vector instructions only need 2 publishers one for the left wheel the other for the right
    std::shared_ptr<rclcpp::Subscription<SubscriptionType>> subscriber_;
    boost::asio::io_service io_;
    boost::asio::serial_port port_;
};



int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto control_node = std::make_shared<MainControlRotor<arrivalType>>();
    executor.add_node(control_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
