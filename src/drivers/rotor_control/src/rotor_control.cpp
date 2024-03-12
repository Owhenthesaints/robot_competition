#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int8.hpp"
#include <iostream>
#include <vector>
#include <memory>



using namespace std::chrono_literals;

/* 
 * @brief This is a higher level class that will handle the controls of the motors 
 * 
 * This class will hopefully offer a very flexible of inputting motor commands. The intended way for now will be 2d vectors transformed into
 * motor commands but through preprocessor directives we leave the door open to other ways
 * */
template<typename MessageType>
class MainControlRotor : public rclcpp::Node
{
public:
    MainControlRotor(int num_children = 2 ) : Node("main_motor_control")
    {
        #ifdef VECTOR_INSTRUCTIONS
        //define two publishers for different topics
        publisher_1_ = this-> create_publisher<MessageType>("motor_updates/m0", 10); 
        publisher_2_ = this-> create_publisher<MessageType>("motor_updates/m1", 10);
        #else
        publisher_vector_();
        for (size_t i =0; i++; i<num_children){
            publisher_vector_.push_back(this-> create_publisher<MessageType>("motor_updates/m"+std::to_string(i), 10));
        }
        #endif
        timer_ = this-> create_wall_timer(500ms, std::bind(&MainControlRotor::timer_callback, this));
    }
private:
    #ifdef VECTOR_INSTRUCTIONS
    void timer_callback(){
        auto wheel1 = MessageType();
        auto wheel2 = MessageType();
        // implement vector logic here
        #ifdef DEBUG
        // placeholder values
        wheel1.data = 25 ;
        wheel2.data = 25;
        #endif // DEBUG
        RCLCPP_INFO(this->get_logger(), "Publishing wheel 0: '%d'", wheel1.data);
        RCLCPP_INFO(this->get_logger(), "Publishing wheel 1: '%d'", wheel2.data);
        publisher_1_-> publish(wheel1);
        publisher_2_-> publish(wheel2);
    }
    #else 
    void timer_callback(){
        auto message = MessageType();
        // to implement
    }
    #endif // VECTOR_INSTRUCTIONS
    rclcpp::TimerBase::SharedPtr timer_;
    // either two publishers for trigo or one vector of publishers
    #ifdef VECTOR_INSTRUCTIONS
    //if the class is built with vector instructions only need 2 publishers one for the left wheel the other for the right
    std::shared_ptr<rclcpp::Publisher<MessageType>> publisher_1_;
    std::shared_ptr<rclcpp::Publisher<MessageType>> publisher_2_;
    #else
    // otherwise have to implement other control scheme
    std::vector<std::shared_ptr<rclcpp::Publisher<MessageType>>> publisher_vector_;
    #endif
};

using std::placeholders::_1;

template<typename MessageType>
class MotorController : public rclcpp::Node 
{
public:
    MotorController() : Node("motor_node_"+std::to_string(id))
    {
        id++;
        subscription_ = this-> create_subscription<MessageType>("motor_updates/m" + std::to_string(id), 10, std::bind(&MotorController::topic_callback, this, _1)); 
    }
    

private:
    void topic_callback(const std::shared_ptr<MessageType> msg) const 
    {
        RCLCPP_INFO(this->get_logger(), "I heard '%d'", msg->data);
    }
    static uint8_t id;
    std::shared_ptr<rclcpp::Subscription<MessageType>> subscription_;
};

template<typename MessageType>
uint8_t MotorController<MessageType>::id = 0;


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController<example_interfaces::msg::Int8>>());
    rclcpp::shutdown();
    return 0;
}
