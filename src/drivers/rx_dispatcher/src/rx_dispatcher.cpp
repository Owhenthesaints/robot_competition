#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <libserial/SerialPort.h>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#define PACKET_LENGTH 10
#define STOP_CHAR 101
#define NUM_IMU_VALS 4
#define NUM_DIST_SENSORS 5

constexpr const uint8_t packetSize = NUM_DIST_SENSORS;

using namespace std::chrono_literals;


template<typename DistSensorPub, typename IMUPub, typename IMUType>
class RxDispatcher : public rclcpp::Node {
public:
    explicit RxDispatcher(const std::string &port = "/dev/ttyACM1", const int &baudRate = 9600,
                          const size_t &ms_timeout = 40) : Node("rx_dispatcher"),
                                                           stored_values(new uint8_t[PACKET_LENGTH - 1]),
                                                           serial(),
                                                           ms_timeout(ms_timeout) {
        // opening serial
        try {
            serial.Open(port);
        } catch (const LibSerial::OpenFailed &) {
            RCLCPP_INFO(this->get_logger(), "failed to open specified file");
            throw;
        }
        if (baudRate == 9600) {
            serial.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        }
        // opening publishers
        distPublisher_ = this->create_publisher<DistSensorPub>("rx/distance/value", 10);
        imuPublisher_ = this->create_publisher<IMUPub>("rx/imu/value", 10);
        // create timer
        timer_ = this->create_wall_timer(25ms, std::bind(&RxDispatcher::timer_callback, this));
    }

private:
    void timer_callback() {
        if (attribute_values()) {
            publishDistSensors();
        }
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
            // find last iterator pointing to stop_char
            while ((it = std::find_if(it, read_buffer.end(), [](int val) { return val == STOP_CHAR; })) !=
                   read_buffer.end()) {
                found = true;
                index = std::distance(read_buffer.begin(), it);
                it++;
            }
        }

        // if found try to imput the values into the class
        if (found) {
            RCLCPP_INFO(this->get_logger(), "distributing values found, packetSize and Index '%s', '%u', '%lu'", found? "true": "false", packetSize, index);
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

    // extra checks should be made here
    /**
     * @brief this function takes in IMUValues and distributes them*/
    bool distributeValuesIMU(LibSerial::DataBuffer IMUValues) {
        uint8_t data[NUM_IMU_VALS * sizeof(IMUType)];
        if (IMUValues.size() == packetSize) {
            std::copy(IMUValues.begin(), IMUValues.end(), data);
        } else {
            return false;
        }
        Converter converter;
        converter.bytes = data;
        std::copy(std::begin(converter.IMUVals), std::end(converter.IMUVals), IMUVal.begin());
        return true;
    };

    union Converter {
        uint8_t bytes[NUM_IMU_VALS * sizeof(IMUType)];
        float IMUVals[NUM_IMU_VALS];
    };

    void publishDistSensors() {
        DistSensorPub msg;
        for (size_t i = 0; i < distSensorArray.size(); i++) {
            msg.data.push_back(distSensorArray[i]);
        }
        distPublisher_->publish(msg);
    };


    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Publisher<DistSensorPub>> distPublisher_;
    std::shared_ptr<rclcpp::Publisher<IMUPub>> imuPublisher_;
    std::unique_ptr<uint8_t[]> stored_values;
    LibSerial::SerialPort serial;
    const size_t ms_timeout;
    std::array<IMUType, NUM_IMU_VALS> IMUVal;
    std::array<uint8_t, NUM_DIST_SENSORS> distSensorArray{};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto rx_node = std::make_shared<RxDispatcher<std_msgs::msg::UInt8MultiArray, sensor_msgs::msg::Imu, float>>();
    rclcpp::spin(rx_node);
    rclcpp::shutdown();
    return 0;
}
