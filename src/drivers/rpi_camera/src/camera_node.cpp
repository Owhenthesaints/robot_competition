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
        // initialise publisher
        image_publisher_ = this->create_publisher<ImageMessageType>("imaging/image_update/image", 10);
        // initialise timer
        timer_ = this->create_wall_timer(1000ms, std::bind(&CameraNode::timer_callback, this));
        // init camera
        cap.open(0);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera");
            exit(0);
        }
    }

    ~CameraNode(){
        cap.release();
    }
private:
    void timer_callback(){
        cv::Mat frame;
        cap>>frame;
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = count_;
        msg_ = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_publisher_->publish(*msg_.get());
        RCLCPP_INFO(this->get_logger(), "Image %ld publisher", count_);
        count_++;
    }

    std::shared_ptr<rclcpp::Publisher<ImageMessageType>> image_publisher_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    std::shared_ptr<ImageMessageType> msg_;
    size_t count_;
    cv::VideoCapture cap;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<CameraNode<sensor_msgs::msg::Image>>());
    rclcpp::shutdown();
}
