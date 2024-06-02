#ifndef MAIN_CONTROLLER_HPP
#define MAIN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

class MainController : public rclcpp::Node {
public:
    explicit MainController();
private:
    void pathing();
    void obstacle_avoidance();
    void return_to_base();
    void go_to_lego();
};



#endif