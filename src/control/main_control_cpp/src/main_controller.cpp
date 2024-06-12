#include "main_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <algorithm>
#include <typeinfo>
#include <boost/numeric/ublas/matrix.hpp>
#include <armadillo>


MainController::MainController() : rclcpp::Node("main_controller"),  backingOutInstruction({std::array<int8_t, 3>({-50, -50, 5}),
                        std::array<int8_t, 3>({30, -30, 3})}), started(true), steadyClock(RCL_STEADY_TIME), startTime(steadyClock.now().seconds()), dropOffLegoDone(false)
{
    state = RobotState::CHOREOGRAPHY;
    legoSubscription = this->create_subscription<legoVisionType>("robot/camera/lego_detected", 10, [this](const legoVisionType::SharedPtr msg)
                                                                 { this->legoDetectionCallback(msg); });
    distanceSensorSubscription = this->create_subscription<distanceType>("rx/distance/value", 10,
                                                                            std::bind(&MainController::distanceCallback, this, std::placeholders::_1));
    carpetSub = this->create_subscription<carpetType>("robot/camera/carpet", 10, [this](const carpetType::SharedPtr msg){ this->carpetCallback(msg);});
    motorCommandSender = this->create_publisher<motorType>("motor_updates/direction", 10);
    baseBeaconSub = this->create_subscription<purpleBeaconType>("robot/camera/purple_beacon", 10,
                                                        [this](const purpleBeaconType::SharedPtr msg){this->purpleBeaconCallback(msg);});
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){ this->mainLoop(); });
    RCLCPP_DEBUG(this->get_logger(), "successfully initiated");
    lastStepChange = steadyClock.now().seconds();
}

inline bool MainController::carpet(){
    return foundCarpetTime + TIME_FORGET_CARPET > steadyClock.now().seconds();
}


bool MainController::followInstructionSet(std::vector<std::array<int8_t, 3>> instructions){
    RCLCPP_DEBUG(this->get_logger(), "in drop off Lego, started instruction is '%s' and in step '%zu' about to send '%d'", startedInstructions? "true": "false", backingOutStep, instructions[backingOutStep][0]);
    double time = steadyClock.now().seconds();
    if (!startedInstructions){
        startedInstructions = true;
        backingOutLastStepTime = steadyClock.now().seconds();
    }
    else if (backingOutStep >= instructions.size()){
        backingOutStep = 0;
        startedInstructions = false;
        return true;
    }

    // if instruction has not expired
    if(static_cast<double>(instructions[backingOutStep][2]) + backingOutLastStepTime > time){
        this->sendCommand(instructions[backingOutStep][0], instructions[backingOutStep][1]);
        RCLCPP_DEBUG(this->get_logger(), "status of time to go to next instruction in backing off '%s'", 
                    instructions[backingOutStep][2] + backingOutLastStepTime < time? "true": "false");
    }
    else{
        backingOutStep++;
        backingOutLastStepTime = steadyClock.now().seconds();
    }
    return false;
}

bool MainController::choreography(){
    return this->followInstructionSet({std::array<int8_t, 3>({100, 100, 15})});
}


/**
 * @brief a function that follows a of predefined instructions to drop off the legos
 */
bool MainController::dropOffLego(){
    return this->followInstructionSet(backingOutInstruction);
}

void MainController::mainLoop(){
    RCLCPP_DEBUG(this->get_logger(), "in main loop with state %d", static_cast<int>(state));
    float time = steadyClock.now().seconds();
    switch(state){
    case RobotState::CHOREOGRAPHY:
        RCLCPP_DEBUG(this->get_logger(), "in choreography in main_loop");
        if(this->choreography())
            this->updateState();
        break;
    case RobotState::STRAIGHT_LINE_NO_CARPET:
    case RobotState::STRAIGHT_LINE:
        RCLCPP_DEBUG(this->get_logger(), "in straight_line in main_loop");
        this->obstacleAvoidance();
        if(time > lastStepChange + TIME_TO_GO_STRAIGHT)
            this->updateState();
        break;
    case RobotState::AIM_FOR_LEGOS:
        if(this->turnToLego())
            this->updateState();
        break;
    case RobotState::AIM_FOR_BEACON:
        RCLCPP_DEBUG(this->get_logger(), "aim for beacon");
        if(this->turnToBeacon())
            this->updateState();
        break;
    case RobotState::DROP_OFF_LEGO:
        if(this->dropOffLego()){
            this->updateState();
        }
        break;
    case RobotState::TURN_AWAY_FROM_CARPET:
        if(this->ninetyDegree())
            this->updateState();
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Pipeline error non existant state");
        break;
    }
}

void MainController::purpleBeaconCallback(const purpleBeaconType::SharedPtr msg){
    foundBeaconTime = steadyClock.now().seconds();
    beaconPosition = {msg->center.position.x, msg->center.position.y};
    inArea = (msg->size_x) > CLOSE_BEACON ? steadyClock.now().seconds(): inArea; // if the beacon is large enough we are in the zone
}

inline bool MainController::timeout(unsigned int timeoutVal)
{
    RCLCPP_DEBUG(this->get_logger(), "timeout ? %s", this->steadyClock.now().seconds() > lastStepChange + static_cast<double>(timeoutVal)? "true" : "false");
    return this->steadyClock.now().seconds() > lastStepChange + static_cast<double>(timeoutVal);
}

bool MainController::turnToBeacon() {
    RCLCPP_DEBUG(this->get_logger(), "in turn to beacon");
    // if beacon is on camera screen 
    double time = steadyClock.now().seconds();
    if(time - foundBeaconTime< BEACON_LOST_TIME){
        if (((beaconPosition[0] <= MIDDLE_BEACON + BEACON_THRESHOLD)&&(beaconPosition[0] >= MIDDLE_BEACON - BEACON_THRESHOLD))){
            RCLCPP_INFO(this->get_logger(), "centered on beacon x position, '%f'", beaconPosition[0]);
            this->sendCommand(0, 0);
            return true;
        }
        else if (beaconPosition[0] > MIDDLE_BEACON + BEACON_THRESHOLD){
            RCLCPP_DEBUG(this->get_logger(), "turning right to beacon: x_position %f", beaconPosition[0]);
            this->slowTurn(false);
        }
        else if (beaconPosition[0] < MIDDLE_BEACON - BEACON_THRESHOLD){
            RCLCPP_DEBUG(this->get_logger(), "turning left to beacon: x_position %f", beaconPosition[0]);
            this->slowTurn(true);
        } else {
            RCLCPP_ERROR(this->get_logger(), "logic error in turn to beacon");
        }
    }
    else if(timeout(BEACON_TIMEOUT)){
        return true;
    }
    else {
        this->slowTurn(true);
    }
    return false;
}

bool MainController::ninetyDegree(){
    return followInstructionSet({std::array<int8_t, 3>({30, -30, 3})});
}

bool MainController::turnToLego()
{
    RCLCPP_DEBUG(this->get_logger(), "in turnToLego");
    // Do not enter function if no legos detected
    if (legoPositions.size()==0){
        return true;
    }
    // get the lowest lego
    size_t lowest_index = 0;
    for (size_t i = 0; i < legoPositions.size(); i++)
    {
        lowest_index = legoPositions.at(lowest_index)[1] > legoPositions.at(i)[1] ? lowest_index : i;
    }

    RCLCPP_DEBUG(this->get_logger(), "lowest index, lowest duplo x, y: %zu, %d, %d", lowest_index, legoPositions.at(lowest_index)[0], legoPositions.at(lowest_index)[1]);

    // try to place it at the middle of the camera
    if(!(legoPositions.at(lowest_index)[0] > -THRESHOLD_MIDDLE + MIDDLE_FRAME && legoPositions.at(lowest_index)[0] < THRESHOLD_MIDDLE + MIDDLE_FRAME))
    {
        if(legoPositions[lowest_index][0] > THRESHOLD_MIDDLE + MIDDLE_FRAME){
            this->slowTurn(false);
        } else if (legoPositions[lowest_index][0] < MIDDLE_FRAME - THRESHOLD_MIDDLE) {
            this->slowTurn(true);
        }
    } else {
        this->sendCommand(0,0);
        return true;
    }
    return false;
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

inline bool MainController::isInArea(){
    return steadyClock.now().seconds() < inArea + LAST_IN_AREA_TIME;
}

void MainController::updateState(){
    double time = steadyClock.now().seconds();
    RCLCPP_DEBUG(this->get_logger(),"carpet state %s", carpet()? "true": "false");
    bool returnToBase = time > startTime + RETURN_TO_BASE_TIME;
    if (carpet() && (!(state==RobotState::STRAIGHT_LINE_NO_CARPET || state==RobotState::CHOREOGRAPHY) || !returnToBase)){
        state = RobotState::TURN_AWAY_FROM_CARPET;
        return;
    }
    lastStepChange = steadyClock.now().seconds();
    // returning to base
    if(returnToBase){
        if (isInArea() && !dropOffLegoDone){
            state = RobotState::DROP_OFF_LEGO;
            dropOffLegoDone = true;
            RCLCPP_INFO(this->get_logger(), "about to get into state DROP_OFF_LEGO");
            return;
        }
        switch(state){
        case RobotState::AIM_FOR_BEACON:
            state = RobotState::STRAIGHT_LINE;
            RCLCPP_INFO(this->get_logger(), "about to get into state STRAIGHT_LINE");
            break;
        case RobotState::STRAIGHT_LINE:
            state = RobotState::AIM_FOR_BEACON;
            RCLCPP_INFO(this->get_logger(), "about to get into state AIM_FOR_BEACON");
            break;
        case RobotState::DROP_OFF_LEGO:
            state = RobotState::STRAIGHT_LINE;
            dropOffLegoDone = false;
            RCLCPP_INFO(this->get_logger(), "dropped off legos");
            startTime = steadyClock.now().seconds();
            break;
        default:
            state = RobotState::AIM_FOR_BEACON;
            RCLCPP_INFO(this->get_logger(), "about to get into state AIM_FOR_BEACON defaulted state is '%d'", static_cast<int>(state));
            break;
        }
        return;
    }

    // normal pivotting state
    switch(state){
    case RobotState::STRAIGHT_LINE:
        state = RobotState::AIM_FOR_LEGOS;
        RCLCPP_INFO(this->get_logger(), "about to get into state AIM_FOR_LEGOS");
        break;
    case RobotState::AIM_FOR_LEGOS:
        state = RobotState::STRAIGHT_LINE;
        RCLCPP_INFO(this->get_logger(), "about to get into state STRAIGHT_LINE");
        break;
    case RobotState::TURN_AWAY_FROM_CARPET:
        state = RobotState::STRAIGHT_LINE;
        RCLCPP_INFO(this->get_logger(), "about to get into state STRAIGHT_LINE from TURN_AWAY_FROM_CARPET");
        break;
    case RobotState::CHOREOGRAPHY:
        state = RobotState::STRAIGHT_LINE_NO_CARPET;
        RCLCPP_INFO(this->get_logger(), "from CHOREOGRAPHY about to go to STRAIGHT_LINE_NO_CARPET");
        break;
    case RobotState::STRAIGHT_LINE_NO_CARPET:
        if(!carpet())
            state = RobotState::STRAIGHT_LINE;
        else
            state = RobotState::STRAIGHT_LINE_NO_CARPET;
        RCLCPP_INFO(this->get_logger(), "from STRAIGHT_LINE_NO_CARPET to STRAIGHT_LINE");
        break;
    default:
        state = RobotState::STRAIGHT_LINE;
        RCLCPP_ERROR(this->get_logger(), "defaulted into unhandled state normally switching back to STRAIGHT_LINE");
        break;
    }
}

void MainController::slowTurn(bool left){
    if (lastCommandHigh > LAST_HIGH_PWM){
        this->sendCommand(0, 0);
        lastCommandHigh= 0;
    } else {
        if (left)
            this->sendCommand(-30, 30); // turn left
        else
            this->sendCommand(30, -30); // turn right
        lastCommandHigh++;
    }
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

void MainController::carpetCallback(const carpetType::SharedPtr msg){
    if ((MIN_CARPET_SIZE_X < msg->size_x) && (MIN_HEIGHT_CARPET < msg->center.position.y)){
        foundCarpetTime = steadyClock.now().seconds();
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
    }
}
