# This code is heavily inspired and some parts are taken from https://github.com/EdjeElectronics/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi/tree/master
# Date: 10/29/22
# This code is subject to the Apache Liscence 2.0

# Import packages
import os
import argparse
import cv2
import numpy as np
import sys
import time
from threading import Thread
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import rclpy
import importlib.util
from vision_msgs.msg import BoundingBox2DArray, Pose2D, Point2D, BoundingBox2D
from std_msgs.msg import Header


class DetectorNode(Node):

    MIN_HEIGHT = 40
    MAX_WIDTH = 30

    def __init__(self, model_name, graph_name, labelmap_name, min_thresh = 0.8, timer_period_legos=0.2, timer_period_beacons=0.1, use_TPU=False):
        # ROS stuff
        super().__init__("lego_detector")
        # declare parameters
        self.declare_parameter('UI', False)
        self.declare_parameter('camNum', 0)
        self._UI = self.get_parameter('UI').get_parameter_value().bool_value
        # declare callback and create publisher for legos
        self._lego_timer = self.create_timer(timer_period_legos, self.find_legos)
        self._lego_publisher = self.create_publisher(BoundingBox2DArray, 'robot/camera/lego_detected', 10)
        # declare callback and create publisher for beacons
        self._beacon_timer = self.create_timer(timer_period_beacons, self.find_purple_beacon)
        self._beacon_publisher = self.create_publisher(BoundingBox2D, 'robot/camera/purple_beacon', 10)

        ### Vision
        # lego model
        # open video stream
        self.video_capture = cv2.VideoCapture(self.get_parameter('camNum').get_parameter_value().integer_value)

        CWD_PATH = get_package_share_directory('vision_py')
        PATH_TO_CKPT = os.path.join(CWD_PATH, model_name, graph_name)
        PATH_TO_LABELS = os.path.join(CWD_PATH, model_name, labelmap_name)

        self.__imW, self.__imH = 1280, 720
        self.use_TPU = use_TPU

        pkg = importlib.util.find_spec('tflite_runtime')
        if pkg:
            from tflite_runtime.interpreter import Interpreter
            if use_TPU:
                from tflite_runtime.interpreter import load_delegate
        else:
            from tensorflow.lite.python.interpreter import Interpreter
            if use_TPU:
                from tensorflow.lite.python.interpreter import load_delegate
        if use_TPU:
            # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
            if (GRAPH_NAME == 'detect.tflite'):
                GRAPH_NAME = 'edgetpu.tflite'

        with open(PATH_TO_LABELS, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]

        # setup the minimum threshold
        self.min_conf_threshold = min_thresh
        # get the working paths
        # define the interpreter
        if use_TPU:
            self.interpreter = Interpreter(model_path=PATH_TO_CKPT,
                                           experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
        else:
            self.interpreter = Interpreter(model_path=PATH_TO_CKPT)

        # allocate the tensors
        self.interpreter.allocate_tensors()

        self.__input_details = self.interpreter.get_input_details()
        self.__output_details = self.interpreter.get_output_details()
        self.__height = self.__input_details[0]['shape'][1]
        self.__width = self.__input_details[0]['shape'][2]

        self.__floating_model = (
            self.__input_details[0]['dtype'] == np.float32)

        self.__input_mean = 127.5
        self.__input_std = 127.5

        # Check output layer name to determine if this model was created with TF2 or TF1,
        # because outputs are ordered differently for TF2 and TF1 models
        outname = self.__output_details[0]['name']

        if ('StatefulPartitionedCall' in outname):  # This is a TF2 model
            self.__boxes_idx, self.__classes_idx, self.__scores_idx = 1, 3, 0
        else:  # This is a TF1 model
            self.__boxes_idx, self.__classes_idx, self.__scores_idx = 0, 1, 2

        ret = self.video_capture.set(3, self.__imH)
        ret = self.video_capture.set(4, self.__imW)
        self.frequency = cv2.getTickFrequency()
        if self._UI:
            self.frame_rate_calc = 1
        
        # beacon model
        min_blue = 90
        min_green = 1
        min_red = 255
        max_blue = 150
        max_green = 4
        max_red = 255
        self.MIN_INTENSITY = 250
        self.MAX_INTENSITY = 255
        self.MIN_COLORS = (min_blue, min_green, min_red)
        self.MAX_COLORS = (max_blue, max_green, max_red)


    def release_cap(self):
        self.video_capture.release()

    def find_legos(self):
        if self._UI:
            t1 = cv2.getTickCount()
        # get frame
        hasFrame, frame1 = self.video_capture.read()

        if not hasFrame:
            self.get_logger.error("cannot get frames")
            return

        # Acquire frame and resize to input shape expected by model [1xHxWx3]
        frame = frame1.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (self.__width, self.__height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.__floating_model:
            input_data = (np.float32(input_data) -
                          self.__input_mean) / self.__input_std

        # Perform detection by running the model with the image as input
        self.interpreter.set_tensor(
            self.__input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Retrieve detection results # Bounding box coordinates of detected objects
        boxes = self.interpreter.get_tensor(self.__output_details[self.__boxes_idx]['index'])[0] 
        # Class index of detected objects
        classes = self.interpreter.get_tensor(self.__output_details[self.__classes_idx]['index'])[0]  
        # Confidence of detected objects
        scores = self.interpreter.get_tensor(self.__output_details[self.__scores_idx]['index'])[0]  

        msg = BoundingBox2DArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        msg.header = header
        
        # Loop over all detections and process each detection if its confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > self.min_conf_threshold) and (scores[i] <= 1.0)):
                self.get_logger().debug("appending boxes")

                # Get bounding box coordinates
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1, (boxes[i][0] * self.__imH)))
                xmin = int(max(1, (boxes[i][1] * self.__imW)))
                ymax = int(min(self.__imH, (boxes[i][2] * self.__imH)))
                xmax = int(min(self.__imW, (boxes[i][3] * self.__imW)))
                bounding_box = BoundingBox2D()
                center = Pose2D()
                position = Point2D()
                position.x = float((xmin + xmax)/2)
                position.y = float((ymin + ymax)/2)
                center.position = position
                center.theta = 0.0
                bounding_box.size_x = float(xmax - xmin)
                bounding_box.size_y = float(ymax - ymin)
                bounding_box.center = center
                msg.boxes.append(bounding_box)


                if self._UI:
                    # Draw bounding box
                    cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                    # Get object's name and draw label
                    object_name = self.labels[int(classes[i])] # Look up object name from "labels" array using class index
                    label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'quarter: 72%'
                    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                    label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                    cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                    cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
        
        if self._UI:
            # Draw framerate in corner of frame
            cv2.putText(frame,'FPS: %.2f' % self.frame_rate_calc,(20,50),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),4,cv2.LINE_AA)
            cv2.putText(frame,'FPS: %.2f' % self.frame_rate_calc,(20,50),cv2.FONT_HERSHEY_PLAIN,2,(230,230,230),2,cv2.LINE_AA)

            # All the results have been drawn on the frame, so it's time to display it.
            cv2.imshow('Object detector', frame)
            cv2.waitKey(1)

        
        self._lego_publisher.publish(msg)

        if self._UI:
            # Calculate framerate
            t2 = cv2.getTickCount()
            time1 = (t2-t1)/self.frequency
            self.frame_rate_calc= 1/time1 
    
    def find_purple_beacon(self):
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

            if w > DetectorNode.MAX_WIDTH or h<DetectorNode.MIN_HEIGHT:
                continue

            # this part of the code indicates that we have found a beacon
            msg = BoundingBox2D()
            msg.center.position.x = float(x+ w/2.)
            msg.center.position.y = float(y + h/2.)
            msg.center.theta = 0.
            msg.size_x = float(w)
            msg.size_y = float(h)
            self._beacon_publisher.publish(msg)
            
            if self._UI:
                cv2.rectangle(color_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.imshow("becon detector", color_img)
                cv2.waitKey(1)
            break


    def __del__(self):
        self.release_cap()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    MODEL_NAME = 'custom_model_lite'
    GRAPH_NAME = 'detect.tflite'
    LABELMAP_NAME = 'labelmap.txt'
    detector = DetectorNode(MODEL_NAME, GRAPH_NAME, LABELMAP_NAME)
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    detector.release_cap()
    detector.destroy_node()
    rclpy.shutdown()
