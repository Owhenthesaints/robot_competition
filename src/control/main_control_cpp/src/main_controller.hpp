#ifndef MAIN_CONTROLLER_HPP
#define MAIN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
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
#define MIDDLE_FRAME 750
#define THRESHOLD_MIDDLE 50
#define TIME_CHANGE_STATE_LOCAL 10
#define TIME_CHANGE_STATE_TO_LEGO 5
#define BEACON_LOST_TIME 1
#define MIDDLE_BEACON 400
#define BEACON_THRESHOLD 50
#define CLOSE_BEACON 20
#define NO_TIME -1
#define LAST_HIGH_PWM 2
#define RETURN_TO_BASE_TIME 50 // seconds
#define TIME_TO_DETECT_BEACON 20
#define TIME_TO_GO_STRAIGHT 5
#define MIN_CARPET_SIZE_X 500
#define TIME_FORGET_CARPET 1 // seconds

enum class RobotState {
    STRAIGHT_LINE,
    AIM_FOR_LEGOS,
    AIM_FOR_BEACON,
    DROP_OFF_LEGO,
    TURN_AWAY_FROM_CARPET,
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
    using purpleBeaconType = vision_msgs::msg::BoundingBox2D;
    using carpetType = purpleBeaconType;
    bool turnToBeacon();
    bool carpet();
    bool ninetyDegree();
    bool dropOffLego();
    bool turnToLego();
    void carpetCallback(const carpetType::SharedPtr msg);
    /**
     * @brief get the positions of lego bricks
    */
    void legoDetectionCallback(const legoVisionType::SharedPtr msg);
    /**
     * @brief get the values of the distance sensors
    */
    void distanceCallback(const distanceType::SharedPtr msg);
    bool followInstructionSet(std::vector<std::array<int8_t, 3>> instructions);
    /**
     * @brief update the states
    */
    void updateState();
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
    void purpleBeaconCallback(const purpleBeaconType::SharedPtr msg);
    void slowTurn(bool left = true);
    std::shared_ptr<rclcpp::Subscription<purpleBeaconType>> baseBeaconSub;
    std::shared_ptr<rclcpp::Subscription<legoVisionType>> legoSubscription;
    std::shared_ptr<rclcpp::Subscription<distanceType>> distanceSensorSubscription;
    std::shared_ptr<rclcpp::Publisher<motorType>> motorCommandSender;
    std::shared_ptr<rclcpp::Subscription<carpetType>> carpetSub;
    std::vector<std::array<int, 2>> legoPositions;
    std::array<uint8_t, NUM_DIST_SENSORS> distanceSensors = {100, 100, 100, 100, 100};
    std::array<bool, NUM_DIST_SENSORS> activatedSensors = {false, false, false, false, false};
    std::array<uint8_t, NUM_DIST_SENSORS> countTracker = {0, 0, 0, 0, 0};
    std::array<double, 2> beaconPosition = {0, 0};
    std::vector<std::array<int8_t, 3>> backingOutInstruction;
    size_t backingOutStep = 0;
    double backingOutLastStepTime = 0;
    bool startedInstructions = false;
    bool inArea = false;
    double foundBeaconTime = NO_TIME;
    bool started = true;
    double foundCarpetTime = NO_TIME;
    std::vector<legoVisionType> legoVision;
    RobotState state = RobotState::STRAIGHT_LINE;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock steadyClock;
    double lastStepChange=0;
    unsigned int lastCommandHigh = 0;
    const double startTime = 0;
    double ninetyDegreeStartTime = 0;
    bool ninetyDegreeBool;
};



#endif
