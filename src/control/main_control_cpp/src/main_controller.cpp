#include "main_controller.hpp"
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <example_interfaces/msg/u_int8_multi_array.hpp>
#include <algorithm>
#include <typeinfo>
#include <boost/numeric/ublas/matrix.hpp>
#include <armadillo>

MainController::MainController() : rclcpp::Node("main_controller")
{
    state = RobotState::STRAIGHT_LINE;
    legoSubscription = this->create_subscription<legoVisionType>("robot/camera/lego_detected", 10, [this](const legoVisionType::SharedPtr msg)
                                                                 { this->legoDetectionCallback(msg); });
    distanceSensorSubscription = this->create_subscription<distanceType>("rx/distance/value", 10, [this](const distanceType::SharedPtr msg)
                                                                         { this->distanceCallback(msg); });
    motorCommandSender = this->create_publisher<motorType>("motor_updates/direction", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){ this->mainLoop(); });
}

void MainController::mainLoop(){
    switch(state){
    case RobotState::STRAIGHT_LINE:
        this->obstacleAvoidance();
        break;
    case RobotState::RETURN_TO_BASE:
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Pipeline error non existant state");
        break;
    }
}

void MainController::sendCommand(int8_t left, int8_t right){
    auto msg = motorType();
    left = MIN(MOTOR_MAX, MAX(left, MOTOR_MIN));
    right = MIN(MOTOR_MAX, MAX(left, MOTOR_MIN));
    msg.data[0] = left;
    msg.data[1] = right;
    this->motorCommandSender->publish(msg);
}

void MainController::obstacleAvoidance(){
    
    arma::vec sensorValues(NUM_DIST_SENSORS);
    arma::vec active(NUM_DIST_SENSORS);

    for (size_t i = 0; i < distanceSensors.size(); ++i){
        sensorValues(i) = static_cast<double>(distanceSensors[i]);
        active(i) = static_cast<double>(activatedSensors[i]);
    }

    sensorValues = 50 - (sensorValues % active);
    

    arma::vec offset = {50, 50};

    arma::mat W = {{1, 1, 1, -1, -1}, {-1, -1, -1, 1, 1}};

    arma::vec motorInputs = (W * sensorValues) + offset;

    this->sendCommand(motorInputs(0), motorInputs(1));
    
}

void MainController::legoDetectionCallback(const legoVisionType::SharedPtr msg){
    
}

void MainController::distanceCallback(const distanceType::SharedPtr msg){
    std::copy(msg->data.begin(), msg->data.begin() + NUM_DIST_SENSORS, distanceSensors.begin());
    auto it = distanceSensors.begin();
    it = std::find_if(it, distanceSensors.end(), [](uint8_t value){return value > DANGER_THRESH;});
    activatedSensors = {false, false, false, false, false};
    while (it!=distanceSensors.end()){
        int index = it - distanceSensors.begin();
        this->activatedSensors[index] = true;
        it = std::find_if(it, distanceSensors.end(), [](uint8_t value){return value > DANGER_THRESH;});
    }
}