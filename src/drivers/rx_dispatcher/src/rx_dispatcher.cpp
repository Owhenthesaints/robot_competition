#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <libserial/SerialPort.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define PACKET_LENGTH 10
#define STOP_CHAR 201
#define NUM_IMU_VALS 4
#define NUM_DIST_SENSORS 5


using namespace std::chrono_literals;

template<typename DistSensorPub, typename IMUPub>
class RxDispatcher : public rclcpp::Node {
public:
    explicit RxDispatcher(const std::string &port = "tty/ACM0", const int & baudRate = 9600, const size_t & ms_timeout = 10) : Node("rx_dispatcher"),
                                                                                       stored_values(new uint8_t[PACKET_LENGTH-1]),
                                                                                       serial(),
                                                                                       ms_timeout(ms_timeout){
        // opening serial
        try {
            serial.Open(port);
        } catch(const LibSerial::OpenFailed&){
            RCLCPP_INFO(this->get_logger(), "failed to open specified file");
        }
        // opening publishers
        dist_publisher_ = this->create_publisher<DistSensorPub>("rx/imu/value", 10);
        imu_publisher_ = this->create_publisher<IMUPub>("rx/imu/value", 10);
        // create timer
        timer_ = this->create_wall_timer(25ms, std::bind(&RxDispatcher::timer_callback, this));
    }

private:
    void timer_callback() {
        // this is an std::vector
        LibSerial::DataBuffer read_buffer;
        bool found = false;
        size_t index = 0;
        try
        {
            serial.Read(read_buffer, 0, ms_timeout);
        }catch (const LibSerial::ReadTimeout &){
            auto it = read_buffer.begin();
            while((it = std::find_if(it, [](int val){return val == STOP_CHAR;}))!= read_buffer.end()){
                found = true;
                index = std::distance(read_buffer.begin(), it);
                it++;
            }
        }
        if (found) {
            if(index < NUM_IMU_VALS - NUM_DIST_SENSORS) {
                if(distributeValuesDistance(LibSerial::DataBuffer(read_buffer.begin() + (int)index - NUM_DIST_SENSORS, read_buffer.begin()+(int)index))){

                }

            }
        }


    }

    /**
     * @brief this function attributes the values to the different sensors
     * @param distanceValues a subvector containing the distance values
     * @return true if successful can add more checks
     */
    bool distributeValuesDistance(std::vector<uint8_t> distanceValues){
        if (distanceValues.size() == NUM_DIST_SENSORS){
            std::copy(distanceValues.begin(), distanceValues.begin() + distSensorArray.size(), distSensorArray.begin());
            return true;
        }
        return false;
    }

    /*
     *  A function to reoder the array and store it inside private store_values
     */

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Publisher<DistSensorPub>> dist_publisher_;
    std::shared_ptr<rclcpp::Publisher<IMUPub>> imu_publisher_;
    std::unique_ptr<uint8_t[]> stored_values;
    LibSerial::SerialPort serial;
    const size_t ms_timeout;
    std::array<float, NUM_IMU_VALS> IMUVal;
    std::array<uint8_t, NUM_DIST_SENSORS> distSensorArray;
};

int main() {
    return 0;
}
