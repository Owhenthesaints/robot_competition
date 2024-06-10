import rclpy
from rclpy.node import Node
import mpu6050
import time
import numpy as np
import os

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu


class IMUPub(Node):
    GRAVITATIONAL_ACCELERATION = 9.81
    PI = 3.14159265358979323846

    def __init__(self):
        super().__init__("IMU_pub")
        # main covariance of the acceleration
        main_cov_accel = (4 * self.GRAVITATIONAL_ACCELERATION * 1e-3)**2
        main_cov_giro = (0.6 * self.PI / 180 * 1e-3)**2
        # setting up all the params
        self.__COVARIANCE_GYRO = np.diag([main_cov_giro, main_cov_giro, main_cov_giro])
        self.__COVARIANCE_ACCEL = np.diag([main_cov_accel, main_cov_accel, main_cov_accel])
        self.publisher_ = self.create_publisher(Imu, 'robot/imu/value', 10)
        # setting up the mpu with the right I2C address
        self.mpu = mpu6050.mpu6050(0x68)
        # creating callback
        timer_period = 0.07
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # get values from giro
        imu_accel = self.mpu.get_accel_data()
        imu_giro = self.mpu.get_gyro_data()
        # setup IMU values
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id="imu_sensor_link"
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
