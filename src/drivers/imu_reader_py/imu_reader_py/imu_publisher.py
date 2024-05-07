import rclpy
from rclpy.node import Node
import mpu6050
import time

from std_msgs.msg import Float32MultiArray

class IMUPub(Node):
    def __init__(self):
        super().__init__("IMU_pub")
        self.publisher_ = self.create_publisher(Float32MultiArray, 'imu/imu/value', 10)
        self.mpu = mpu6050.mpu6050(0x68)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def read_sensors(self):
        IMU_accel = list(self.mpu.get_accel_data().values())
        IMU_giro = list(self.mpu.get_gyro_data().values())
        # could add temp but who cares about temp
        return IMU_accel + IMU_giro

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = list(self.read_sensors())
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    IMU_publisher = IMUPub()

    rclpy.spin(IMU_publisher)

    IMU_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
