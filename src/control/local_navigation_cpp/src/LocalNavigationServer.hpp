//
// Created by owhenthesaints on 21/05/24.
//

#ifndef BUILD_LOCAL_NAVIGATION_SERVER_HPP
#define BUILD_LOCAL_NAVIGATION_SERVER_HPP
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>
#include "local_navigation_cpp/action/local_navigation.hpp"


class LocalNavigationServer : public rclcpp::Node {
public:
    LocalNavigationServer();
    using LocalNavigationMessage = local_navigation_cpp::action::LocalNavigation;
private:
    rclcpp_action::Server<LocalNavigationMessage>::SharedPtr server_;


};


#endif //BUILD_LOCAL_NAVIGATION_SERVER_HPP
