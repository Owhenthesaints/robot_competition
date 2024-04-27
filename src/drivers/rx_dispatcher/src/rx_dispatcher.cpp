#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <utility>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define PACKET_LENGTH 10
#define STOP_CHAR 201


using namespace std::chrono_literals;

template<typename DistSensorPub, typename IMUPub>
class RxDispatcher : public rclcpp::Node {
public:
    explicit RxDispatcher(const std::string &port = "tty/ACM0", int baudRate = 9600) : Node("rx_dispatcher"), io(),
                                                                                       serial(io, port),
                                                                                       stored_values(new uint8_t[PACKET_LENGTH-1]) {
        dist_publisher_ = this->create_publisher<DistSensorPub>("rx/imu/value", 10);
        imu_publisher_ = this->create_publisher<IMUPub>("rx/imu/value", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&RxDispatcher::timer_callback, this));
        serial.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
    }

private:
    void timer_callback() {
        using namespace boost;
        auto values = std::make_unique<uint8_t[]>(PACKET_LENGTH);
        while (true) {
            asio::read(serial, asio::buffer(&values, PACKET_LENGTH));
            for (uint8_t i = 0; i < PACKET_LENGTH; i++) {
                if (values[i] == STOP_CHAR) {
                    circular_read(values, i);
                    return;
                }
            }
        }
    }

    /*
     *  A function to reoder the array and store it inside private store_values
     */
    void circular_read(std::unique_ptr<uint8_t[]> &values, uint8_t begin_point) {
        for (uint8_t i = begin_point; i!= begin_point - 1; i == PACKET_LENGTH-1 ? i = 0 : i++) {
            stored_values[i] = values[i];
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Publisher<DistSensorPub>> dist_publisher_;
    std::shared_ptr<rclcpp::Publisher<IMUPub>> imu_publisher_;
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    std::unique_ptr<uint8_t[]> stored_values;
};

int main() {
    return 0;
}
