import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

class BalanceController(Node):
    def __init__(self):
        super().__init__('pid_sim')

        # Declare parameters
        self.declare_parameter('Kp', 13.0)
        self.declare_parameter('Ki', 2.0)
        self.declare_parameter('Kd', 4.0)
        self.declare_parameter('output_limit', 2.0)

        # Get parameter values
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.output_limit = self.get_parameter('output_limit').get_parameter_value().double_value

        # PID state variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

        # ROS interfaces
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(f'PID node initialized with Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}')

    def imu_callback(self, msg: Imu):
        # Extract pitch from quaternion
        q = msg.orientation
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        # PID error
        error = pitch  # Inverted to balance

        current_time = time.time()
        dt = current_time - self.prev_time if self.prev_time else 0.01

        # PID computation
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Clamp output to avoid overshooting
        output = max(-self.output_limit, min(self.output_limit, output))

        # Publish command
        cmd = Twist()
        cmd.linear.x = output
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)

        # Update previous state
        self.prev_error = error
        self.prev_time = current_time

        # Log for debugging
        self.get_logger().debug(f"Pitch: {math.degrees(pitch):.2f}, Output: {output:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
