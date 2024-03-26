#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/Image.hpp"

using namespace std;


template<class ImageMessageType>
class CameraNode: public Node {
public:
    CameraNode(): Node("camera_node"){
        image_publisher_ = this->create_publisher<ImageMessageType>("imaging/image_update/image", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&CameraNode::timer_callback, this));
    }
private:
    void timer_callback(){
        auto message = ImageMessageType();
        message.data = 
    }
    rclcpp::Publisher<ImageMessageType>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(){

    cv::VideoCapture cap(0);

    if(!cap.isOpened()) {
        cout<<"Error: Couldn't open camera" <<endl;
        return -1;
    }
    
    cv::Mat frame;
    
    cap>>frame;

    if (frame.empty()) {
        cout<<"Error: Couldn't capture frame"<<endl;
        return -1;
    }

    cap.release();


    string directory = string(getenv("HOME"))+ "/captured_image.jpg";

    cv::imwrite(directory, frame);

    return 0;

}
