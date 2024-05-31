import rclpy
from rclpy.node import Node
import mpu6050
import time
import numpy as np
import os

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu



class IMUPub(Node):
    def __init__(self):
        super().__init__("IMU_pub")
        self.__COVARIANCE = np.array([[1.267995719792864911e-03,-4.086623843803083289e-05,2.659803543383608697e-05,6.984727682528767915e-05,-4.621744718599019106e-05,6.864952701665839483e-04],
                             [-4.086623843803083289e-05,2.219589922458633895e-03,2.535032644819837808e-05,3.952182696245238350e-05,2.985988930803461903e-05,1.622096735774475806e-03],
                             [2.659803543383608697e-05,2.535032644819837808e-05,2.475723331447657067e-03,-1.238537177369436579e-04,-3.395396012921391836e-05,-9.655998567511241831e-04],
                             [6.984727682528767915e-05,3.952182696245238350e-05,-1.238537177369436579e-04,9.306212368354672623e-03,-6.645253814496823990e-04,1.678109912451748167e-03],
                             [-4.621744718599019106e-05,2.985988930803461903e-05,-3.395396012921391836e-05,-6.645253814496823990e-04,1.311478930992902685e-02,1.443720006968440264e-03],
                             [6.864952701665839483e-04,1.622096735774475806e-03,-9.655998567511241831e-04,1.678109912451748167e-03,1.443720006968440264e-03,6.247741041263324968e-01]])
        self.publisher_ = self.create_publisher(Imu, 'imu/robot/value', 10)
        self.mpu = mpu6050.mpu6050(0x68)
        timer_period = 0.07
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # get values from giro
        imu_accel = self.mpu.get_accel_data()
        imu_giro = self.mpu.get_gyro_data()
        # setup IMU values
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id="base_link"
        # setup the imu accel values
        imu_msg.linear_acceleration.x = imu_accel['x']
        imu_msg.linear_acceleration.y = imu_accel['y']
        imu_msg.linear_acceleration.z = imu_accel['z']
        imu_msg.linear_acceleration_covariance = self.__COVARIANCE[0: 3, 0: 3].flatten()
        # setup the gyro data
        imu_msg.angular_velocity.x = imu_giro['x']
        imu_msg.angular_velocity.y = imu_giro['y']
        imu_msg.angular_velocity.z = imu_giro['z']
        imu_msg.angular_velocity_covariance = self.__COVARIANCE[3:, 3:].flatten()



        self.publisher_.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    IMU_publisher = IMUPub()

    rclpy.spin(IMU_publisher)

    IMU_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
