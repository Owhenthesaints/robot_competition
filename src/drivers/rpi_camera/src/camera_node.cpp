#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

using namespace std;


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

    cap.release();

    string directory = cv::getenv("HOME")+ "/captured_image.jpg";

    cv::imwrite(directory, frame);

    return 0;

}
