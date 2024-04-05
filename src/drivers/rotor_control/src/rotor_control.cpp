#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int8.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"
#include <wiringPi.h>
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

// left
#define PWM_PIN_LEFT 23
// right
#define PWM_PIN_RIGHT 26
//left dir 
#define WHEEL_DIR_LEFT 21
//right dir
#define WHEEL_DIR_RIGHT 22
//left enable
#define WHEEL_ENABLE_LEFT 15
//right enable
#define WHEEL_ENABLE_RIGHT 16
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
template<typename MessageType, typename SubscriptionType>
class MainControlRotor : public rclcpp::Node
{
public:
    MainControlRotor(int num_children = 2 ) : Node("main_motor_control")
    {
        // message of DEBUG severity
        RCLCPP_DEBUG(this-> get_logger(), "starting init MainControlRotor");
        timer_ = this-> create_wall_timer(50ms, std::bind(&MainControlRotor::timer_callback, this));
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
        wheel1.data = MIN(MAX(wheel_array_[0], -MAX_SPEED_CST), MAX_SPEED_CST);
        wheel2.data = MIN(MAX(wheel_array_[1], -MAX_SPEED_CST), MAX_SPEED_CST);
#ifdef DEBUG
        // placeholder values
        RCLCPP_INFO(this->get_logger(), "debug mod wheels always have the same value");
        wheel1.data = 25;
        wheel2.data = 25;
#endif // ENDOF_DEBUG
        RCLCPP_INFO(this->get_logger(), "Publishing wheel 0: '%d'", wheel1.data);
        RCLCPP_INFO(this->get_logger(), "Publishing wheel 1: '%d'", wheel2.data);
        publisher_1_-> publish(wheel1);
        publisher_2_-> publish(wheel2);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    // wheel_array_[1] up down, joystick_array_[2] left right
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
    MotorController(uint8_t pwm_choice, uint8_t dir_choice, uint8_t enable_choice) : Node("motor_node_"+std::to_string(id)), dir_pin_(dir_choice), enable_pin_(enable_choice), pwm_pin_(pwm_choice)
    {
        // wiringPi setup
        if(wiringPiSetup()==-1)
            exit(10);
        pinMode(pwm_choice, PWM_OUTPUT);
        pinMode(dir_choice, OUTPUT);
        pinMode(enable_choice, OUTPUT); 
        // initialising motors
        RCLCPP_DEBUG(this->get_logger(), "initialising Motor Controller '%d'", id);
        subscription_ = this-> create_subscription<SubscriptionType>("motor_updates/m" + std::to_string(id), 10, std::bind(&MotorController::topic_callback, this, _1)); 
        id++;
    }

    ~MotorController() {
        pinMode(pwm_pin_, INPUT);
        pinMode(dir_pin_, INPUT);
        pinMode(enable_pin_, INPUT);
    }
    

private:
    void topic_callback(const std::shared_ptr<SubscriptionType> msg) const 
    {
        RCLCPP_INFO(this->get_logger(), "writing to pin '%d'", pwm_pin_);
        int8_t speed = msg->data;
        uint16_t  pwmValue= std::abs(std::floor(speed * CONVERSION_CONSTANT));

        // speed threshold
        if (speed < THRESHOLD and speed > -THRESHOLD)
        {
            digitalWrite(enable_pin_, LOW);
        }
        //backwards forwards
        else if(speed<0){
            digitalWrite(dir_pin_, LOW);
            digitalWrite(enable_pin_, HIGH);
        }
        else if(speed>0) {
            digitalWrite(dir_pin_, HIGH);
            digitalWrite(enable_pin_, HIGH);
        }

        RCLCPP_INFO(this->get_logger(), "callback writing '%d'", pwmValue);
        pwmWrite(pwm_pin_, pwmValue);
    }
    static uint8_t id;
    std::shared_ptr<rclcpp::Subscription<SubscriptionType>> subscription_;
    uint8_t pwm_pin_;
    uint8_t dir_pin_;
    uint8_t enable_pin_;
};

template<typename SubscriptionType>
uint8_t MotorController<SubscriptionType>::id = 0;


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto control_node = std::make_shared<MainControlRotor<comType, arrivalType>>();
    auto motor_node_0 = std::make_shared<MotorController<comType>>(PWM_PIN_LEFT, WHEEL_DIR_LEFT, WHEEL_ENABLE_LEFT);
    auto motor_node_1 = std::make_shared<MotorController<comType>>(PWM_PIN_RIGHT, WHEEL_DIR_RIGHT, WHEEL_ENABLE_RIGHT);
    executor.add_node(control_node);
    executor.add_node(motor_node_0);
    executor.add_node(motor_node_1);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
