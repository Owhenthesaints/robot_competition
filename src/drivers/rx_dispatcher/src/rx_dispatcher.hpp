#ifndef RX_DISPATCHER_HPP
#define RX_DISPATCHER_HPP

#include <rclcpp/rclcpp.hpp>

#define PACKET_LENGTH 10
#define STOP_CHAR 101
#define NUM_DIST_SENSORS 5
#define CALLBACK_TIME 50ms // ms

constexpr const uint8_t packetSize = NUM_DIST_SENSORS;
using namespace std::chrono_literals;


template<typename DistSensorPub, typename RotorControlSub>
class RxDispatcher : public rclcpp::Node {
public:
    explicit RxDispatcher(const std::string &port = "/dev/ttyACM0", const int &baudRate = 9600,
                          const size_t &ms_timeout = 40) : Node("rx_dispatcher"),
                                                           stored_values(new uint8_t[PACKET_LENGTH - 1]),
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
        }
        // opening publishers
        distPublisher_ = this->create_publisher<DistSensorPub>("rx/distance/value", 10);
        rotorSubscription_ = this->create_subscription<RotorControlSub>(
                "motor_updates/direction", 10,
                std::bind(&RxDispatcher::rotor_values_updates, this, std::placeholders::_1));
        // create timer
        timer_ = this->create_wall_timer(CALLBACK_TIME, std::bind(&RxDispatcher::timer_callback, this));
    }

private:
    void timer_callback() {
        if (attribute_values()) {
            publishDistSensors();
        }
        send_motor_msgs();
    }

    void rotor_values_updates(const RotorControlSub & msg){
        RCLCPP_INFO(this->get_logger(), "updating motor, values with '%d' and '%d'", msg.data[0], msg.data[1]);
        motorArray_[0] = msg.data[0];
        motorArray_[1] = msg.data[1];
    }

    void send_motor_msgs(){
        std::string msg;
        msg += motorArray_[0];
        msg += motorArray_[1];
        msg += char(STOP_CHAR);
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
            RCLCPP_INFO(this->get_logger(), "What is buffer size'%lu'", read_buffer.size());
            // find last iterator pointing to stop_char
            while ((it = std::find_if(it, read_buffer.end(), [](int val) { return val == STOP_CHAR; })) !=
                   read_buffer.end()) {
                RCLCPP_INFO(this->get_logger(), "trying to find");
                found = true;
                index = std::distance(read_buffer.begin(), it);
                it++;
            }
        }

        // if found try to imput the values into the class
        if (found) {
            RCLCPP_INFO(this->get_logger(), "distributing values found, packetSize and Index '%s', '%u', '%lu'",
                        found ? "true" : "false", packetSize, index);
            if (index >= packetSize) {
                RCLCPP_INFO(this->get_logger(), "distributing values");
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
    std::unique_ptr<uint8_t[]> stored_values;
    LibSerial::SerialPort serial;
    const size_t ms_timeout;
    std::array<uint8_t, NUM_DIST_SENSORS> distSensorArray{};
    int8_t motorArray_[2]={0, 0};
};


#endif