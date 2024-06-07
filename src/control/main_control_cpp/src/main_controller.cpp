#include "main_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <algorithm>
#include <typeinfo>
#include <boost/numeric/ublas/matrix.hpp>
#include <armadillo>


MainController::MainController() : rclcpp::Node("main_controller"), started(true), steadyClock(RCL_STEADY_TIME)
{
    state = RobotState::STRAIGHT_LINE;
    legoSubscription = this->create_subscription<legoVisionType>("robot/camera/lego_detected", 10, [this](const legoVisionType::SharedPtr msg)
                                                                 { this->legoDetectionCallback(msg); });
    distanceSensorSubscription = this->create_subscription<distanceType>("rx/distance/value", 10,
                                                                            std::bind(&MainController::distanceCallback, this, std::placeholders::_1));
    motorCommandSender = this->create_publisher<motorType>("motor_updates/direction", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){ this->mainLoop(); });
    RCLCPP_DEBUG(this->get_logger(), "successfully initiated");
    lastStepChange = steadyClock.now().seconds();
}

void MainController::mainLoop(){
    RCLCPP_DEBUG(this->get_logger(), "in main loop");
    float time = steadyClock.now().seconds();
    switch(state){
    case RobotState::STRAIGHT_LINE:
        this->obstacleAvoidance();
        if (time>lastStepChange + TIME_CHANGE_STATE_LOCAL){
            state = RobotState::AIM_FOR_LEGOS;
            lastStepChange = steadyClock.now().seconds();
            this->sendCommand(0, 0);
        }
        break;
    case RobotState::AIM_FOR_LEGOS:
        this->turnToLego();
        if(time>lastStepChange + TIME_CHANGE_STATE_TO_LEGO){
            state = RobotState::STRAIGHT_LINE;
            lastStepChange = steadyClock.now().seconds();
            this->sendCommand(0, 0);
        }
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Pipeline error non existant state");
        break;
    }
}

void MainController::turnToLego()
{
    RCLCPP_DEBUG(this->get_logger(), "in turnToLego");
    // Do not enter function if no legos detected
    if (legoPositions.size()==0){
        return;
    }
    size_t lowest_index = 0;
    for (size_t i = 0; i < legoPositions.size(); i++)
    {
        lowest_index = legoPositions.at(lowest_index)[1] > legoPositions.at(i)[1] ? lowest_index : i;
    }

    RCLCPP_DEBUG(this->get_logger(), "lowest index, lowest duplo x, y: %zu, %d, %d", lowest_index, legoPositions.at(lowest_index)[0], legoPositions.at(lowest_index)[1]);


    if(!(legoPositions.at(lowest_index)[0] > -THRESHOLD_MIDDLE + MIDDLE_FRAME && legoPositions.at(lowest_index)[0] < THRESHOLD_MIDDLE + MIDDLE_FRAME))
    {
        if(legoPositions[lowest_index][0] > THRESHOLD_MIDDLE + MIDDLE_FRAME){
            this->sendCommand(30, -30);
        } else if (legoPositions[lowest_index][0] < MIDDLE_FRAME - THRESHOLD_MIDDLE) {
            this->sendCommand(-30, 30);
        }
    } else {
        this->sendCommand(0,0);
    }
}

void MainController::sendCommand(int8_t left, int8_t right){
    auto msg = motorType();
    left = MIN(MOTOR_MAX, MAX(left, MOTOR_MIN));
    right = MIN(MOTOR_MAX, MAX(right, MOTOR_MIN));
    msg.data.push_back(left);
    msg.data.push_back(right);
    this->motorCommandSender->publish(msg);
}

void MainController::obstacleAvoidance(){
    
    arma::vec sensorValues(NUM_DIST_SENSORS);
    arma::vec active(NUM_DIST_SENSORS);

    for (size_t i = 0; i < distanceSensors.size(); ++i){
        sensorValues(i) = static_cast<double>(distanceSensors[i]);
        active(i) = static_cast<double>(activatedSensors[i]);
    }

    sensorValues = (50 - sensorValues) % active;

    RCLCPP_DEBUG(this->get_logger(), "sensorValues 0  %.2f", sensorValues(0));
    RCLCPP_DEBUG(this->get_logger(), "sensorValue 1 %.2f", sensorValues(2));

    arma::vec offset = {50, 50};

    arma::mat W = {{1, 1, 1, -1, -1}, {-1, -1, -1, 1, 1}};

    arma::vec motorInputs = (W * sensorValues) + offset;

    RCLCPP_DEBUG(this->get_logger(), "motor updates %.2f, %.2f", motorInputs(0), motorInputs(1));

    RCLCPP_DEBUG(this->get_logger(), "finished arma operations obstacle avoidance");

    this->sendCommand(static_cast<int8_t>(motorInputs(0)), static_cast<int8_t>(motorInputs(1)));
    
}

void MainController::legoDetectionCallback(const legoVisionType::SharedPtr msg){
    legoPositions.clear();
    for(size_t i(0); i<msg->boxes.size(); i++){
        if(msg->boxes[i].size_x > MAX_BOX_SIZE){
            continue;
        }
        else {
            legoPositions.push_back(std::array<int, 2>({static_cast<int>(msg->boxes[i].center.position.x), static_cast<int>(msg->boxes[i].center.position.y)}));
        }
    }
}

void MainController::distanceCallback(const distanceType::SharedPtr msg)
{
    // Simple code to descide to reduce noise it asks if the captors have had detection three times in a row
    activatedSensors = {false, false, false, false, false};
    for (size_t i(0); i < activatedSensors.size(); i++)
    {
        distanceSensors[i] = msg->data[i];
        if (started && (distanceSensors[i] <= 50)) {
            if (countTracker[i] > 3){
                activatedSensors[i] = true;
            }
            else
                countTracker[i]++;
        }
        else if (started)
        {
            countTracker[i] = 0;
            activatedSensors[i] = false;
        }
        RCLCPP_DEBUG(this->get_logger(), "in iteration %zu, value inside sensor %u", i, distanceSensors[i]);
    }
}
