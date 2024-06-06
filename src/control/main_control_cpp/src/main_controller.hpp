#ifndef MAIN_CONTROLLER_HPP
#define MAIN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <example_interfaces/msg/int8_multi_array.hpp>
#include <vector>

#define NUM_DIST_SENSORS 5
#define DANGER_THRESH 50
#define MOTOR_MIN -100
#define MOTOR_MAX 100
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MAX_BOX_SIZE 500
#define MIDDLE_FRAME 700
#define THRESHOLD_MIDDLE 150
#define TIME_CHANGE_STATE_LOCAL 10
#define TIME_CHANGE_STATE_TO_LEGO 5


enum class RobotState {
    STRAIGHT_LINE,
    AIM_FOR_LEGOS,
};

class MainController : public rclcpp::Node {
public:
    MainController();
private:
    /**
     * @brief main callback function (our state machine)
    */
    void mainLoop();
    using legoVisionType = vision_msgs::msg::BoundingBox2DArray;
    using distanceType = std_msgs::msg::UInt8MultiArray;
    using motorType = example_interfaces::msg::Int8MultiArray;
    void pathing();
    void turnToLego();
    /**
     * @brief get the positions of lego bricks
    */
    void legoDetectionCallback(const legoVisionType::SharedPtr msg);
    /**
     * @brief get the values of the distance sensors
    */
    void distanceCallback(const distanceType::SharedPtr msg);
    /**
     * @brief send a command to the motors
     * @param left left motor value [-100;100]
     * @param right right motor value [-100;100]
    */
    void sendCommand(int8_t left, int8_t right);
    /**
     * @brief go straight with local navigation
    */
    void obstacleAvoidance();
    std::shared_ptr<rclcpp::Subscription<legoVisionType>> legoSubscription;
    std::shared_ptr<rclcpp::Subscription<distanceType>> distanceSensorSubscription;
    std::shared_ptr<rclcpp::Publisher<motorType>> motorCommandSender;
    std::vector<std::array<int, 2>> legoPositions;
    std::array<uint8_t, NUM_DIST_SENSORS> distanceSensors = {100, 100, 100, 100, 100};
    std::array<bool, NUM_DIST_SENSORS> activatedSensors = {false, false, false, false, false};
    std::array<uint8_t, NUM_DIST_SENSORS> countTracker = {0, 0, 0, 0, 0};
    bool started = true;
    std::vector<legoVisionType> legoVision;
    RobotState state = RobotState::STRAIGHT_LINE;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock steadyClock;
    float lastStepChange=0;
};



#endif
