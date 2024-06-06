from rclpy import Node
from vision_msgs.msg import BoundingBox2DArray
import cv2
import rclpy


class BeaconDetector(Node):
    def __init__(self):
        super().__init__("beacon_detector")
        self.declare_parameter('camNum', 0)
        timer_period = 0.25
        self.publishers_ = self.create_publisher(BoundingBox2DArray, "robot/camera/beacon")
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.video_capture = cv2.VideoCapture(self.get_parameter('camNum').get_parameter_value().integer_value)

        min_blue = 0
        min_green = 0
        min_red = 255
        max_green = 0
        max_blue = 0
        max_red = 255
        self.MIN_INTENSITY = 255
        self.MAX_INTENSITY = 255
        self.MIN_COLORS = (min_blue, min_green, min_red)
        self.MAX_COLORS = (max_blue, max_green, max_red)



    def timer_callback(self):
        r, img = self.video_capture.read()
        if not r:
            return
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


        mask = cv2.inRange(hsv_img, self.MIN_COLORS, self.MAX_COLORS)
        
        mask2 = cv2.inRange(gray_scale, self.MIN_INTENSITY, self.MAX_INTENSITY)

        mask3 = cv2.bitwise_and(mask, mask2)
        
        cv2.imshow("becon detector", mask3)
        cv2.waitKey(1)

    def __del__(self):
        cv2.destroyAllWindows()
        

        
        
        

def main(args):
    rclpy.init(args=args)
    rclpy.spin(BeaconDetector())
    rclpy.shutdown()