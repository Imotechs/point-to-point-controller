#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class MoveRobot(Node):
#     def __init__(self):
#         super().__init__('circle_mover')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.get_logger().info('CircleMover node started.')

#     def timer_callback(self):
#         msg = Twist()
#         msg.linear.x = 0.2
#         msg.angular.z = 0.2  # combine both to move in a circle
#         self.publisher_.publish(msg)
#         self.get_logger().info("moving robot...")

# def main(args=None):
#     rclpy.init(args=args)
#     node = MoveRobot()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import math
import random
import matplotlib.pyplot as plt
import threading
from matplotlib.animation import FuncAnimation

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.1, self.move_robot)
        self.current_pose = Pose2D()

        # Random goal
        rand_x = random.uniform(-7.5, 7.5)
        rand_y = random.uniform(-7.5, 7.5)
        self.goal = Pose2D(x=rand_x, y=rand_y, theta=0.0)

        # Angular PID terms
        self.kp_ang = 1.5
        self.ki_ang = 0.0
        self.kd_ang = 0.3
        self.prev_ang_err = 0.0
        self.int_ang_err = 0.0

        # Linear PID terms
        self.kp_lin = 0.5
        self.ki_lin = 0.0
        self.kd_lin = 0.1
        self.prev_lin_err = 0.0
        self.int_lin_err = 0.0

        # PID Metrics for live plotting
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.time_data = []
        self.distance_data = []
        self.linear_x_data = []
        self.angle_error_data = []
        self.angular_z_data = []

        # Start live plotting in a thread
        threading.Thread(target=self.start_plotting, daemon=True).start()

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal.x
        marker.pose.position.y = self.goal.y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

    def move_robot(self):
        self.publish_goal_marker()

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        dx = self.goal.x - self.current_pose.x
        dy = self.goal.y - self.current_pose.y
        distance = math.hypot(dx, dy)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(angle_to_goal - self.current_pose.theta),
                                 math.cos(angle_to_goal - self.current_pose.theta))

        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        # Phase 1: Orientation Correction
        if abs(angle_error) > 0.1:
            self.int_ang_err += angle_error
            derivative = angle_error - self.prev_ang_err
            ang_output = (
                self.kp_ang * angle_error +
                self.ki_ang * self.int_ang_err +
                self.kd_ang * derivative
            )
            msg.twist.angular.z = max(min(ang_output, 1.5), -1.5)
            msg.twist.linear.x = 0.0
            self.prev_ang_err = angle_error

            # Log for plotting
            self.time_data.append(current_time)
            self.angle_error_data.append(angle_error)
            self.angular_z_data.append(msg.twist.angular.z)
            self.distance_data.append(distance)
            self.linear_x_data.append(0.0)

        # Phase 2: Linear Movement
        elif distance > 0.05:
            self.int_lin_err += distance
            derivative = distance - self.prev_lin_err
            lin_output = (
                self.kp_lin * distance +
                self.ki_lin * self.int_lin_err +
                self.kd_lin * derivative
            )
            msg.twist.linear.x = max(min(lin_output, 0.5), 0.0)
            msg.twist.angular.z = 0.0
            self.prev_lin_err = distance

            self.time_data.append(current_time)
            self.angle_error_data.append(angle_error)
            self.angular_z_data.append(0.0)
            self.distance_data.append(distance)
            self.linear_x_data.append(msg.twist.linear.x)

        else:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0

        self.publisher.publish(msg)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_pose.x = position.x
        self.current_pose.y = position.y
        q = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        self.current_pose.theta = yaw

    def start_plotting(self):
        plt.ion()
        fig, axs = plt.subplots(2, 1, figsize=(10, 6))

        def update_plot(_):
            if len(self.time_data) < 2:
                return
            axs[0].clear()
            axs[1].clear()

            axs[0].plot(self.time_data, self.distance_data, label='Distance')
            axs[0].plot(self.time_data, self.linear_x_data, label='Linear Velocity')
            axs[0].set_title("Linear PID")
            axs[0].legend()
            axs[0].grid(True)

            axs[1].plot(self.time_data, self.angle_error_data, label='Angle Error')
            axs[1].plot(self.time_data, self.angular_z_data, label='Angular Velocity')
            axs[1].set_title("Angular PID")
            axs[1].legend()
            axs[1].grid(True)
            axs[1].set_xlabel("Time (s)")

            plt.tight_layout()

        ani = FuncAnimation(fig, update_plot, interval=200)
        plt.show(block=True)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

