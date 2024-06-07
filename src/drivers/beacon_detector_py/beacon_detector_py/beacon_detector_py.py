from rclpy.node import Node
from vision_msgs.msg import BoundingBox2DArray
import cv2
import rclpy
import numpy as np


class BeaconDetector(Node):

    MIN_HEIGHT = 40
    MAX_WIDTH = 30

    def __init__(self):
        super().__init__("beacon_detector")
        self.declare_parameter('camNum', 0)
        self.declare_parameter('UI', False)
        timer_period = 0.25
        self.publishers_ = self.create_publisher(BoundingBox2DArray, "robot/camera/beacon", 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.video_capture = cv2.VideoCapture(self.get_parameter('camNum').get_parameter_value().integer_value)
        self.__UI = self.get_parameter('UI').get_parameter_value().bool_value

        min_blue = 90
        min_green = 1
        min_red = 255
        max_blue = 150
        max_green = 8
        max_red = 255
        self.MIN_INTENSITY = 250
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

        # Find all connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask3, connectivity=8)

        # The first component is the background, so we start from the second component
        areas = stats[1:, cv2.CC_STAT_AREA]
        sorted_indices = np.argsort(areas)[::-1]
        for index in sorted_indices:
            analysed_component = 1 + index

            # Create a new mask for the largest connected component
            largest_component_mask = np.zeros_like(mask3)
            largest_component_mask[labels == analysed_component] = 255

            x = stats[analysed_component, cv2.CC_STAT_LEFT]
            y = stats[analysed_component, cv2.CC_STAT_TOP]
            w = stats[analysed_component, cv2.CC_STAT_WIDTH]
            h = stats[analysed_component, cv2.CC_STAT_HEIGHT]

            color_img = cv2.cvtColor(mask3, cv2.COLOR_GRAY2BGR)

            if w > BeaconDetector.MAX_WIDTH or h<BeaconDetector.MIN_HEIGHT:
                continue

            
            if self.__UI:
                cv2.rectangle(color_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.imshow("becon detector", color_img)
                cv2.waitKey(1)
                break

    def __del__(self):
        cv2.destroyAllWindows()
        

        
        
        

def main(args = None):
    rclpy.init(args=args)
    rclpy.spin(BeaconDetector())
    rclpy.shutdown()