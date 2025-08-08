#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial as pyserial  # Changed import name
import ujson
import math

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('broadcastor_imu')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0') #/dev/ttyACM0
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('child_frame_id', 'imu_orientation')
        
        # Setup serial connection
        self.serial = pyserial.Serial(  # Changed to pyserial
            self.get_parameter('serial_port').value,
            self.get_parameter('baudrate').value,
            timeout=1
        )
        
        # Create publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for reading serial data
        self.create_timer(0.01, self.read_serial_data)  # 100Hz
        
        # Complementary filter variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info("IMU Publisher and TF Broadcaster started")

    def complementary_filter(self, ax, ay, az, gx, gy, gz, dt):
        """
        Complementary filter to estimate orientation from
        accelerometer and gyroscope data
        """
        # Constants
        ALPHA = 0.98
        
        # Calculate roll and pitch from accelerometer
        roll_acc = math.atan2(ay, math.sqrt(ax**2 + az**2))
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        # Integrate gyroscope data
        self.roll = ALPHA * (self.roll + gx * dt) + (1 - ALPHA) * roll_acc
        self.pitch = ALPHA * (self.pitch + gy * dt) + (1 - ALPHA) * pitch_acc
        self.yaw += gz * dt  # Yaw from gyro only (no magnetometer)
        
        return self.roll, self.pitch, self.yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw

    def read_serial_data(self):
        if self.serial.in_waiting:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                data = ujson.loads(line)
                
                # Get current time
                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds / 1e9
                self.last_time = current_time
                
                # Get IMU data
                ax = data['accel']['x']
                ay = data['accel']['y']
                az = data['accel']['z']
                gx = data['gyro']['x']
                gy = data['gyro']['y']
                gz = data['gyro']['z']
                
                # Apply complementary filter to get orientation
                roll, pitch, yaw = self.complementary_filter(ax, ay, az, gx, gy, gz, dt)
                qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
                
                # Create and publish IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = current_time.to_msg()
                imu_msg.header.frame_id = self.get_parameter('frame_id').value
                
                # Linear acceleration (m/s²)
                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az
                
                # Angular velocity (rad/s)
                imu_msg.angular_velocity.x = gx
                imu_msg.angular_velocity.y = gy
                imu_msg.angular_velocity.z = gz
                
                # Orientation (quaternion)
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw
                
                # Set covariance (identity matrix)
                imu_msg.linear_acceleration_covariance[0] = 0.01
                imu_msg.linear_acceleration_covariance[4] = 0.01
                imu_msg.linear_acceleration_covariance[8] = 0.01
                
                imu_msg.angular_velocity_covariance[0] = 0.01
                imu_msg.angular_velocity_covariance[4] = 0.01
                imu_msg.angular_velocity_covariance[8] = 0.01
                
                imu_msg.orientation_covariance[0] = 0.01
                imu_msg.orientation_covariance[4] = 0.01
                imu_msg.orientation_covariance[8] = 0.01
                
                self.imu_pub.publish(imu_msg)
                
                # Broadcast TF transform
                t = TransformStamped()
                t.header.stamp = imu_msg.header.stamp
                t.header.frame_id = self.get_parameter('frame_id').value
                t.child_frame_id = self.get_parameter('child_frame_id').value
                
                # Set rotation from quaternion
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                
                # Send transform
                self.tf_broadcaster.sendTransform(t)
                
                # Log data for debugging
                self.get_logger().info(
                    f"Accel: ({ax:.3f}, {ay:.3f}, {az:.3f}) | "
                    f"Gyro: ({gx:.3f}, {gy:.3f}, {gz:.3f}) | "
                    f"Orientation: ({math.degrees(roll):.1f}°, "
                    f"{math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°)",
                    throttle_duration_sec=1.0
                )
                
            except Exception as e:
                self.get_logger().warn(f"Error processing IMU data: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()