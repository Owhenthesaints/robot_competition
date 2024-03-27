#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <memory>

using namespace std::chrono_literals;


template<class ImageMessageType>
class CameraNode: public rclcpp::Node {
public:
    CameraNode(): rclcpp::Node("camera_node"){
        image_publisher_ = this->create_publisher<ImageMessageType>("imaging/image_update/image", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&CameraNode::timer_callback, this));
    }
private:
    void timer_callback(){
        cv::Mat my_image(cv::Size(640, 480), CV_8UC3);
        cv::randu(my_image, cv::Scalar(0,0,0), cv::Scalar(255, 255, 255));
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image).toImageMsg();
        image_publisher_->publish(*msg_.get());
        RCLCPP_INFO(this->get_logger(), "Image %ld publisher", count_);
        count_++;
    }
    std::shared_ptr<rclcpp::Publisher<ImageMessageType>> image_publisher_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    std::shared_ptr<ImageMessageType> msg_;
    size_t count_;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<CameraNode<sensor_msgs::msg::Image>>());
    rclcpp::shutdown();
}
