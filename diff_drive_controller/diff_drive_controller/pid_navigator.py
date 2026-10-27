import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class DiffDriveNavigator(Node):
    def __init__(self):
        super().__init__('diff_drive_navigator')
        
        # State variables
        self.has_target = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Robot physical specs
        self.wheel_base = 1.2
        self.wheel_radius = 0.4
        self.max_linear_velocity = 5.0 * self.wheel_radius
        
        # PID Initialization
        self.linear_pid = PIDController(kp=0.5, ki=0.0, kd=0.1)
        self.angular_pid = PIDController(kp=1.5, ki=0.01, kd=0.2)
        
        self.last_time = self.get_clock().now()
        
        # Publishers and Subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/orange_bot/odometry', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Navigator ready. Waiting for /goal_pose...")

    def goal_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.has_target = True
        self.get_logger().info(f"New target received: X: {self.target_x}, Y: {self.target_y}")

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def control_loop(self):
        if not self.has_target:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0 or dt > 0.1:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance_error = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        if distance_error < 0.1:
            self.stop_robot()
            self.has_target = False
            self.get_logger().info("Target Reached! Waiting for next goal...")
            return

        v = self.linear_pid.compute(distance_error, dt)
        w = self.angular_pid.compute(yaw_error, dt)

        if abs(yaw_error) > math.pi / 4:
            v = 0.0  

        v_right = v + (w * self.wheel_base / 2.0)
        v_left = v - (w * self.wheel_base / 2.0)
        max_requested = max(abs(v_right), abs(v_left))

        if max_requested > self.max_linear_velocity:
            scale_factor = self.max_linear_velocity / max_requested
            v *= scale_factor
            w *= scale_factor
            self.linear_pid.integral -= distance_error * dt
            self.angular_pid.integral -= yaw_error * dt

        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = DiffDriveNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()