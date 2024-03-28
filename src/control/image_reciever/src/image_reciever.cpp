#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <string>

using std::placeholders::_1;

template <typename ImageType>
class ImageReciever : public rclcpp::Node
{
public:
    ImageReciever() : Node("image_reciever"), count_(0)
    {
        reciever_ = this->create_subscription<ImageType>("imaging/image_update/image", 10, std::bind(&ImageReciever::topic_callback, this, _1));
        //setup home dir
        homeDir = std::string(std::getenv("HOME"));
        homeDir += "/Documents/courses/robot_competition/images/";
    }

private:
    void topic_callback(const ImageType &msg)
    {
        RCLCPP_INFO(this->get_logger(), "inside callback");
        try
        {
            lastImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        catch (cv_bridge::Exception & e){
            RCLCPP_INFO(this->get_logger(), "Could not convert from '%s' to 'brg8'.", msg.encoding.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "recieved_image");
        cv::imshow("recieved_image", lastImage);
        cv::waitKey(0.5);
    }
    cv::Mat lastImage;
    std::shared_ptr<rclcpp::Subscription<ImageType>> reciever_;
    std::string homeDir;
    size_t count_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageReciever<sensor_msgs::msg::Image>>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}