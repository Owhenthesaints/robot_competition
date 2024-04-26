#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

template <typename DistSensorPub, typename IMUPub>
class RxDispatcher : public rclcpp::Node
{
public:
    RxDispatcher() : Node("rx_dispatcher"){
        dist_publisher_ = this->create_publisher<DistSensorPub>("rx/imu/value", 10);
        imu_publisher_ = this->create_publisher<IMUPub>("rx/imu/value", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
private:
    void timer_callback() {

        
    }
}
