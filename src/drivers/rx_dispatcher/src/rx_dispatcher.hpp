#ifndef RX_DISPATCHER_HPP
#define RX_DISPATCHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
#include <numbers>
#include <vector>

#define PACKET_LENGTH 10
#define STOP_CHAR 101
#define NUM_DIST_SENSORS 5
#define CALLBACK_TIME 50ms    // ms
#define ROBOT_WIDTH 0.357     // m
#define ROBOT_WIDTH_2 0.34    // m
#define DIAMETER_WHEEL 0.1142 // m
#define ONEHUNDRED_TO_RPM 4000
#define REDUCTION_RATIO 60
#define PWM_CUTOFF 28
//
#define PI 3.14159265358979323846
constexpr double WHEEL_SPEED_CONVERSION_TO_RAD = ONEHUNDRED_TO_RPM / (double)100 / (double)60 * 2 * PI / (double)REDUCTION_RATIO;

constexpr const uint8_t packetSize = NUM_DIST_SENSORS;
using namespace std::chrono_literals;


template<typename DistSensorPub, typename RotorControlSub>
class RxDispatcher : public rclcpp::Node {
public:
    explicit RxDispatcher(const std::string &port = "/dev/ttyACM0", const int &baudRate = 9600,
                          const size_t &ms_timeout = 20) : Node("rx_dispatcher"),
                                                           stored_values(new uint8_t[packetSize - 1]),
                                                           serial(),
                                                           ms_timeout(ms_timeout) {
        // opening serial
        try {
            serial.Open(port);
        } catch (const LibSerial::OpenFailed &) {
            RCLCPP_ERROR(this->get_logger(), "failed to open specified port");
            throw;
        }
        if (baudRate == 9600) {
            serial.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        } else if (baudRate == 19200) {
            serial.SetBaudRate(LibSerial::BaudRate::BAUD_19200);
        }
        // opening publishers
        distPublisher_ = this->create_publisher<DistSensorPub>("rx/distance/value", 10);
        speedPublisher_ = this-> create_publisher<speedPublisherType>("robot/speed/value", 10);
        rotorSubscription_ = this->create_subscription<RotorControlSub>(
                "motor_updates/direction", 10,
                std::bind(&RxDispatcher::rotor_values_updates, this, std::placeholders::_1));
        // create timer
        timer_ = this->create_wall_timer(CALLBACK_TIME, std::bind(&RxDispatcher::timer_callback, this));
    }

private:
    using speedPublisherType = nav_msgs::msg::Odometry;
    void timer_callback() {
        if (attribute_values()) {
            publishDistSensors();
        }
        updateSpeedAndSendMessages();
    }

    void rotor_values_updates(const RotorControlSub & msg){
        RCLCPP_INFO(this->get_logger(), "updating motor, values with '%d' and '%d'", msg.data[0], msg.data[1]);
        motorArray_[0] = msg.data[0];
        motorArray_[1] = msg.data[1];
    }

    void sendMotorMsgs(){
        std::string msg;
        msg += motorArray_[0];
        msg += motorArray_[1];
        msg += char(STOP_CHAR);
        serial.FlushIOBuffers();
        serial.Write(msg);
    }

    /**
     * @brief a function that handles the reading from serial and attributes the values
     */
    bool attribute_values() {

        // this is an std::vector
        LibSerial::DataBuffer read_buffer;
        bool found = false;
        size_t index = 0;
        try {
            serial.Read(read_buffer, 0, ms_timeout);
        } catch (const LibSerial::ReadTimeout &) {
            auto it = read_buffer.begin();
            RCLCPP_DEBUG(this->get_logger(), "What is buffer size'%lu'", read_buffer.size());
            // find last iterator pointing to stop_char
            while ((it = std::find_if(it, read_buffer.end(), [](int val) { return val == STOP_CHAR; })) !=
                   read_buffer.end()) {
                RCLCPP_DEBUG(this->get_logger(), "trying to find");
                found = true;
                index = std::distance(read_buffer.begin(), it);
                it++;
            }
        }

        // if found try to imput the values into the class
        if (found) {
            RCLCPP_INFO(this->get_logger(), "distributing values found, read_buffer size and Index '%s', '%lu', '%lu'",
                        found ? "true" : "false", read_buffer.size(), index);
            if (index >= packetSize) {
                RCLCPP_DEBUG(this->get_logger(), "distributing values");
                distributeValuesDistance(LibSerial::DataBuffer(
                        read_buffer.begin() + (int) index - packetSize,
                        read_buffer.begin() + (int) index));
                return true;
            }
        }
        return false;
    }

    /**
     * @brief this function attributes the values to the different sensors
     * @param distanceValues a subvector containing the distance values
     * @return true if successful can add more checks
     */
    bool distributeValuesDistance(LibSerial::DataBuffer distanceValues) {
        if (distanceValues.size() == NUM_DIST_SENSORS) {
            std::copy(distanceValues.begin(), distanceValues.begin() + distSensorArray.size(), distSensorArray.begin());
            return true;
        }
        return false;
    }

    /**
     * @brief update the speeds for the kallman node
     */
    void updateSpeed()
    {
        auto msg = speedPublisherType();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        // Check if the motors are above arduino cutoff
        int16_t left = static_cast<int16_t>(motorArray_[0]>PWM_CUTOFF || motorArray_[0]< -PWM_CUTOFF? motorArray_[0]: 0);
        int16_t right = static_cast<int16_t>(motorArray_[1]>PWM_CUTOFF || motorArray_[1]< -PWM_CUTOFF? motorArray_[1]: 0);
        int16_t difference = right - left;
        msg.twist.twist.angular.z = difference / (double)2 * DIAMETER_WHEEL / 2 * WHEEL_SPEED_CONVERSION_TO_RAD / (ROBOT_WIDTH / 2);
        msg.twist.twist.linear.x = DIAMETER_WHEEL / 2 * WHEEL_SPEED_CONVERSION_TO_RAD * ((left + right) / (double)2); // get the speed of the motor
        msg.twist.twist.linear.y = 0;
        msg.twist.twist.linear.z = 0;
        // these are the value of group 2 from last year
        msg.twist.covariance = std::array<double, 36>({
            0.05, 0, 0, 0, 0, 0,    // Variance in x (0.05^2)
            0, 0.05, 0, 0, 0, 0,    // Variance in y (0.05^2)
            0, 0, 0.05, 0, 0, 0,    // Variance in z (0.01^2)
            0, 0, 0, 0.1, 0, 0, // Variance in roll (0.01^2)
            0, 0, 0, 0, 0.1, 0,    // Variance in pitch (0.01^2)
            0, 0, 0, 0, 0, 0.1     // Variance in yaw (0.02^2)
        });
        speedPublisher_->publish(msg);
    }

    /**
     * @brief the function helps streemline the callback calls sendMotorMsgs and updateSpeed
    */
    void updateSpeedAndSendMessages() {
        this-> sendMotorMsgs();
        this-> updateSpeed();
    }



    void publishDistSensors() {
        DistSensorPub msg;
        for (unsigned char & i : distSensorArray) {
            msg.data.push_back(i);
        }
        distPublisher_->publish(msg);
    };


    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Publisher<DistSensorPub>> distPublisher_;
    std::shared_ptr<rclcpp::Subscription<RotorControlSub>> rotorSubscription_;
    std::shared_ptr<rclcpp::Publisher<speedPublisherType>> speedPublisher_;
    std::unique_ptr<uint8_t[]> stored_values;
    LibSerial::SerialPort serial;
    const size_t ms_timeout;
    std::array<uint8_t, NUM_DIST_SENSORS> distSensorArray{};
    int8_t motorArray_[2]={0, 0};
};


#endif
