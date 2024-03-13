#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int8.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"
#include <iostream>
#include <vector>
#include <memory>

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
        wheelArray = new int8_t[num_children];
#ifdef VECTOR_INSTRUCTIONS
        //define two publishers for different topics
        RCLCPP_DEBUG(this-> get_logger(), "starting vector instruction implementation init");
        wheelArray[0] = 0;
        wheelArray[1] = 0;
        publisher_1_ = this-> create_publisher<MessageType>("motor_updates/m0", 10); 
        publisher_2_ = this-> create_publisher<MessageType>("motor_updates/m1", 10);
        subscriber_ = this-> create_subscription<SubscriptionType>("motor_updates/direction", 10, std::bind(&MainControlRotor::rotor_values_update, this, _1));
#else
        RCLCPP_DEBUG(this -> get_logger(), "starting non vector instruction init");
        publisher_vector_();
        for (size_t i =0; i++; i<num_children){
            publisher_vector_.push_back(this-> create_publisher<MessageType>("motor_updates/m"+std::to_string(i), 10));
        }
#endif
    }
private:
#ifdef VECTOR_INSTRUCTIONS
    void rotor_values_update(std::shared_ptr<SubscriptionType> msg){
        RCLCPP_DEBUG(this-> get_logger(), "updated wheel values");
        wheelArray[0] = msg->data[0];
        wheelArray[1] = msg->data[1];
    }
    void timer_callback(){
        RCLCPP_DEBUG(this->get_logger(), "entering timer callback");
        auto wheel1 = MessageType();
        auto wheel2 = MessageType();
        // implement vector logic here
#ifdef DEBUG
        // placeholder values
        RCLCPP_DEBUG(this->get_logger(), "debug mod wheels always have the same value");
        wheel1.data = wheelArray[0];
        wheel2.data = wheelArray[1];
#endif // ENDOF_DEBUG
        RCLCPP_INFO(this->get_logger(), "Publishing wheel 0: '%d'", wheel1.data);
        RCLCPP_INFO(this->get_logger(), "Publishing wheel 1: '%d'", wheel2.data);
        publisher_1_-> publish(wheel1);
        publisher_2_-> publish(wheel2);
    }
#else // ENDOF_VECTOR_INSTRUCTIONS
    void timer_callback(){
        auto message = MessageType();
        // to implement
    }
#endif // ENDOF_NOT_VECTOR_INSTRUCTIONS
    rclcpp::TimerBase::SharedPtr timer_;
    int8_t * wheelArray;
    // either two publishers for trigo or one vector of publishers
#ifdef VECTOR_INSTRUCTIONS
    //if the class is built with vector instructions only need 2 publishers one for the left wheel the other for the right
    std::shared_ptr<rclcpp::Publisher<MessageType>> publisher_1_;
    std::shared_ptr<rclcpp::Publisher<MessageType>> publisher_2_;
    std::shared_ptr<rclcpp::Subscription<SubscriptionType>> subscriber_;
#else // ENDOF_VECTOR_INSTRUCTIONS
    // otherwise have to implement other control scheme
    std::vector<std::shared_ptr<rclcpp::Publisher<MessageType>>> publisher_vector_;
#endif // ENDOF_NOT_VECTOR_INTRUCTIONS
};


template<typename SubscriptionType>
class MotorController : public rclcpp::Node 
{
public:
    MotorController() : Node("motor_node_"+std::to_string(id))
    {
        RCLCPP_DEBUG(this->get_logger(), "initialising Motor Controller '%d'", id);
        id++;
        subscription_ = this-> create_subscription<SubscriptionType>("motor_updates/m" + std::to_string(id), 10, std::bind(&MotorController::topic_callback, this, _1)); 
    }
    

private:
    void topic_callback(const std::shared_ptr<SubscriptionType> msg) const 
    {
        RCLCPP_INFO(this->get_logger(), "I heard '%d'", msg->data);
    }
    static uint8_t id;
    std::shared_ptr<rclcpp::Subscription<SubscriptionType>> subscription_;
};

template<typename SubscriptionType>
uint8_t MotorController<SubscriptionType>::id = 0;


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController<comType>>());
    rclcpp::shutdown();
    return 0;
}
