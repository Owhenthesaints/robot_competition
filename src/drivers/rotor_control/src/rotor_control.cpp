#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int8.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"
#include <wiringPi.h>
#include <iostream>
#include <vector>
#include <memory>

#define PWM_PIN_1 23
#define PWM_PIN_2 26


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
template<typename MessageType, typename SubscriptionType>
class MainControlRotor : public rclcpp::Node
{
public:
    MainControlRotor(int num_children = 2 ) : Node("main_motor_control")
    {
        // message of DEBUG severity
        RCLCPP_DEBUG(this-> get_logger(), "starting init MainControlRotor");
        timer_ = this-> create_wall_timer(500ms, std::bind(&MainControlRotor::timer_callback, this));
        wheel_array_ = new int8_t[num_children];
        //define two publishers for different topics
        RCLCPP_DEBUG(this-> get_logger(), "starting vector instruction implementation init");
        wheel_array_[0] = 0;
        wheel_array_[1] = 0;
        publisher_1_ = this-> create_publisher<MessageType>("motor_updates/m0", 10); 
        publisher_2_ = this-> create_publisher<MessageType>("motor_updates/m1", 10);
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
        auto wheel1 = MessageType();
        auto wheel2 = MessageType();
        // implement vector logic here
#ifdef DEBUG
        // placeholder values
        RCLCPP_DEBUG(this->get_logger(), "debug mod wheels always have the same value");
        wheel1.data = 25;
        wheel2.data = 25;
#endif // ENDOF_DEBUG
        RCLCPP_INFO(this->get_logger(), "Publishing wheel 0: '%d'", wheel1.data);
        RCLCPP_INFO(this->get_logger(), "Publishing wheel 1: '%d'", wheel2.data);
        publisher_1_-> publish(wheel1);
        publisher_2_-> publish(wheel2);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int8_t * wheel_array_;
    //if the class is built with vector instructions only need 2 publishers one for the left wheel the other for the right
    std::shared_ptr<rclcpp::Publisher<MessageType>> publisher_1_;
    std::shared_ptr<rclcpp::Publisher<MessageType>> publisher_2_;
    std::shared_ptr<rclcpp::Subscription<SubscriptionType>> subscriber_;
};


/*
 * @brief a class to control the different motors individually
 *
 * A class which automatically controls a number of different motors affected to each pin*/
template<typename SubscriptionType>
class MotorController : public rclcpp::Node 
{
public:
    MotorController(uint8_t pwm_choice) : Node("motor_node_"+std::to_string(id))
    {
        RCLCPP_DEBUG(this->get_logger(), "initialising Motor Controller '%d'", id);
        subscription_ = this-> create_subscription<SubscriptionType>("motor_updates/m" + std::to_string(id), 10, std::bind(&MotorController::topic_callback, this, _1)); 
        id++;
        pwm_pin_ = pwm_choice;
    }
    

private:
    void topic_callback(const std::shared_ptr<SubscriptionType> msg) const 
    {
        RCLCPP_INFO(this->get_logger(), "writing to pin '%d'", pwm_pin_);
        uint16_t  pwmValue= std::floor(msg->data * 10);
        RCLCPP_INFO(this->get_logger(), "callback writing '%d'", pwmValue);
        pwmWrite(pwm_pin_, pwmValue);

    }
    static uint8_t id;
    std::shared_ptr<rclcpp::Subscription<SubscriptionType>> subscription_;
    uint8_t pwm_pin_;
};

template<typename SubscriptionType>
uint8_t MotorController<SubscriptionType>::id = 0;


int main(int argc, char * argv[]){
    if(wiringPiSetup()==-1)
        exit(10);
    pinMode(PWM_PIN_1, PWM_OUTPUT);
    pinMode(PWM_PIN_2, PWM_OUTPUT);
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto control_node = std::make_shared<MainControlRotor<comType, arrivalType>>();
    auto motor_node_0 = std::make_shared<MotorController<comType>>(PWM_PIN_1);
    auto motor_node_1 = std::make_shared<MotorController<comType>>(PWM_PIN_2);
    executor.add_node(control_node);
    executor.add_node(motor_node_0);
    executor.add_node(motor_node_1);
    executor.spin();
    rclcpp::shutdown();
    pinMode(PWM_PIN_1, INPUT);
    pinMode(PWM_PIN_2, INPUT);
    return 0;
}
