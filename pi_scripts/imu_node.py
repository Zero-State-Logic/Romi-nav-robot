#!/usr/bin/env python3
import smbus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

MPU = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

def to_signed(val):
    return val - 65536 if val >= 0x8000 else val

class ImuNode(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(MPU, PWR_MGMT_1, 0)
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.05, self.publish_imu)
        self.errors = 0
        self.get_logger().info('MPU6050 node started')

    def publish_imu(self):
        try:
            data = self.bus.read_i2c_block_data(MPU, ACCEL_XOUT_H, 14)
        except OSError:
            self.errors += 1
            if self.errors % 20 == 0:
                self.get_logger().warn(f'I2C read glitch (count={self.errors})')
            return
        ax = to_signed((data[0] << 8) | data[1]) / 16384.0 * 9.80665
        ay = to_signed((data[2] << 8) | data[3]) / 16384.0 * 9.80665
        az = to_signed((data[4] << 8) | data[5]) / 16384.0 * 9.80665
        gx = to_signed((data[8] << 8) | data[9]) / 131.0 * 0.0174533
        gy = to_signed((data[10] << 8) | data[11]) / 131.0 * 0.0174533
        gz = to_signed((data[12] << 8) | data[13]) / 131.0 * 0.0174533
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.orientation_covariance[0] = -1.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
