import mpu6050
import time
import numpy as np

mpu6050 = mpu6050.mpu6050(0x68)

def read_sensor_data():
    accelerometer_data = mpu6050.get_accel_data()

    gyroscope_data = mpu6050.get_gyro_data()

    temperature = mpu6050.get_temp()

    return list(accelerometer_data.values()) + list(gyroscope_data.values())

if __name__ == "__main__":
    array = np.empty((0, 6))
    timestamp = time.time()
    while True:
        array = np.vstack([array, read_sensor_data()])
        if time.time() - timestamp > 10:
            break
    np.savetxt('imu_vals_10s_arr.csv', array, delimiter=',')
