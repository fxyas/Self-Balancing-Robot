import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

class BalanceController(Node):
    def __init__(self):
        super().__init__('pid_sim')

        # PID gains
        self.Kp = 5.0
        self.Ki = 0.1
        self.Kd = 0.

        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

        # Subscriptions and publications
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('pid_sim initialized.')

    def imu_callback(self, msg: Imu):
        # Convert quaternion to pitch
        q = msg.orientation
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        # PID error is pitch angle (robot should stay vertical => pitch = 0)
        error = 0.0 - pitch

        current_time = time.time()
        dt = current_time - self.prev_time if self.prev_time else 0.01

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = output  # Use x-axis velocity to correct tilt
        cmd.angular.z = 0.0

        self.publisher.publish(cmd)

        # Store previous values
        self.prev_error = error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
