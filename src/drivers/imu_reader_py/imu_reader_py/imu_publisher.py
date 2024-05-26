import rclpy
from rclpy.node import Node
import mpu6050
import time

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

class IMUPub(Node):
    def __init__(self):
        super().__init__("IMU_pub")
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
        # setup the gyro data
        imu_msg.angular_velocity.x = imu_giro['x']
        imu_msg.angular_velocity.y = imu_giro['y']
        imu_msg.angular_velocity.z = imu_giro['z']



        self.publisher_.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    IMU_publisher = IMUPub()

    rclpy.spin(IMU_publisher)

    IMU_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
