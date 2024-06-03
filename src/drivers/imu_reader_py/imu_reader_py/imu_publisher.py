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
        self.__COVARIANCE_GYRO = np.diag([5.4732e-04, 6.1791e-04, 6.2090e-04])
        self.__COVARIANCE_ACCEL = np.diag([2.8e-03, 2.5e-03, 3.8e-03])
        self.publisher_ = self.create_publisher(Imu, 'robot/imu/value', 10)
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
        imu_msg.linear_acceleration_covariance = self.__COVARIANCE_ACCEL.flatten()
        # setup the gyro data
        imu_msg.angular_velocity.x = imu_giro['x']
        imu_msg.angular_velocity.y = imu_giro['y']
        imu_msg.angular_velocity.z = imu_giro['z']
        imu_msg.angular_velocity_covariance = self.__COVARIANCE_GYRO.flatten()



        self.publisher_.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    IMU_publisher = IMUPub()

    rclpy.spin(IMU_publisher)

    IMU_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
