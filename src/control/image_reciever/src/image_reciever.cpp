#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

using std::placeholders::_1;

template<typename ImageType>
class ImageReciever : public Node
{
public:
    ImageReciever() : Node("image_reciever")
    {
        reciever_ = this-> create_subscription<ImageType>("camera/updates/get_image", 10, std::bind(&ImageReciever::topic_callback, this, _1)) 
    }
private:
    void topic_callback(const ImageType & msg){
        
    }

private:
    rclcpp::Subscription<ImageType> reciever_;
};

int main(void){
    
}