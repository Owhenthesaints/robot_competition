//
// Created by owhenthesaints on 21/05/24.
//

#include "LocalNavigationServer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

LocalNavigationServer::LocalNavigationServer() : rclcpp::Node("local_navigation_server") {
    server_ = rclcpp_action::create_server<LocalNavigationMessage>(this, "local_navigation",
                                                                   [this](const rclcpp_action::GoalUUID &uuid,
                                                                          std::shared_ptr<const LocalNavigationMessage::Goal> goal) {
                                                                       return this->handle_goal(uuid, goal);
                                                                   },
                                                                   [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<LocalNavigationMessage>> goal_handle) {
                                                                       return this->handle_cancel(goal_handle);
                                                                   },
                                                                   [this](const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
                                                                       this->handle_accepted(goal_handle);
                                                                   });
}
