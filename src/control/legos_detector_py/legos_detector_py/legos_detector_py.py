######## Count Change Using Object Detection #########
#
# Author: Evan Juras, EJ Technology Consultants (www.ejtech.io)
# Date: 10/29/22
#  
# Description:
# This program uses a TFLite coin detection model to locate and identify coins in 
# a live camera feed. It calculates the total value of the coins in the camera's view.
# (Works on US currency, but can be modified to work with coins from other countries!)

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


class LegoDetector(Node):
    
    def __init__(self, model_name, graph_name, labelmap_name, timer_period = 0.01, use_TPU = False):
        super().__init__("lego_detector")
        self.timer_period = timer_period
        self.timer = self.create_timer(timer_period, self.analyse_vision)
        self.video_capture = cv2.VideoCapture(0)

        ### Vision
        CWD_PATH = get_package_share_directory('legos_detector_py')
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

        self.min_conf_threshold = 0.5
        #get the working paths 
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
        
        self.__floating_model = (self.__input_details[0]['dtype'] == np.float32)

        self.__input_mean = 127.5
        self.__input_std = 127.5

        # Check output layer name to determine if this model was created with TF2 or TF1,
        # because outputs are ordered differently for TF2 and TF1 models
        outname = self.__output_details[0]['name']

        if ('StatefulPartitionedCall' in outname): # This is a TF2 model
            self.__boxes_idx, self.__classes_idx, self.__scores_idx = 1, 3, 0
        else: # This is a TF1 model
            self.__boxes_idx, self.__classes_idx, self.__scores_idx = 0, 1, 2

        ret = self.video_capture.set(3, self.__imH)
        ret = self.video_capture.set(4, self.__imW)
        self.frequency = cv2.getTickFrequency()
        self.frame_rate_calc = 1
    
    def release_cap(self):
        self.video_capture.release()
    
    def test(self):
        ret, frame = self.video_capture.read()
        if ret:
            cv2.imshow("Beautiful Feed", frame)
            
    

    def analyse_vision(self):
        t1 = cv2.getTickCount()

        # get frame
        hasFrame, frame1 = self.video_capture.read()

        # Acquire frame and resize to input shape expected by model [1xHxWx3]
        frame = frame1.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (self.__width, self.__height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.__floating_model:
            input_data = (np.float32(input_data) - self.__input_mean) / self.__input_std

        # Perform detection by running the model with the image as input
        self.interpreter.set_tensor(self.__input_details[0]['index'],input_data)
        self.interpreter.invoke()

        # Retrieve detection results
        boxes = self.interpreter.get_tensor(self.__output_details[self.__boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = self.interpreter.get_tensor(self.__output_details[self.__classes_idx]['index'])[0] # Class index of detected objects
        scores = self.interpreter.get_tensor(self.__output_details[self.__scores_idx]['index'])[0] # Confidence of detected objects

        # Loop over all detections and process each detection if its confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > self.min_conf_threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * self.__imH)))
                xmin = int(max(1,(boxes[i][1] * self.__imW)))
                ymax = int(min(self.__imH,(boxes[i][2] * self.__imH)))
                xmax = int(min(self.__imW,(boxes[i][3] * self.__imW)))

                # Draw bounding box
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                # Get object's name and draw label
                object_name = self.labels[int(classes[i])] # Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'quarter: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text


        # Draw framerate in corner of frame
        cv2.putText(frame,'FPS: %.2f' % self.frame_rate_calc,(20,50),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),4,cv2.LINE_AA)
        cv2.putText(frame,'FPS: %.2f' % self.frame_rate_calc,(20,50),cv2.FONT_HERSHEY_PLAIN,2,(230,230,230),2,cv2.LINE_AA)

        # All the results have been drawn on the frame, so it's time to display it.
        cv2.imshow('Object detector', frame)
        cv2.waitKey(1)

        # Calculate framerate
        t2 = cv2.getTickCount()
        time1 = (t2-t1)/self.frequency
        self.frame_rate_calc= 1/time1


    def __del__(self):
        self.release_cap()
        cv2.destroyAllWindows()

        

def main(args = None):
    rclpy.init(args = args)
    MODEL_NAME = 'custom_model_lite'
    GRAPH_NAME = 'detect.tflite'
    LABELMAP_NAME = 'labelmap.txt'
    detector = LegoDetector(MODEL_NAME, GRAPH_NAME, LABELMAP_NAME)
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    detector.release_cap()
    detector.destroy_node()
    rclpy.shutdown()
