#!/usr/bin/env python3

import csv
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class VelocityTest(Node):

    def __init__(self):
        super().__init__('velocity_test')

        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        filename = time.strftime(
            '/tmp/velocity_test_%Y%m%d_%H%M%S.csv'
        )

        self.file = open(filename, 'w', newline='')

        self.csv_writer = csv.writer(self.file)

        self.csv_writer.writerow([
            'simulation_time',
            'commanded_velocity',
            'actual_velocity',
            'actual_acceleration',
            'robot_x',
            'robot_y'
        ])

        self.filename = filename

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.previous_time = None
        self.previous_velocity = None

        self.actual_velocity = 0.0
        self.actual_acceleration = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.timer = self.create_timer(0.05, self.control)

        self.get_logger().info(
            f'Logging to {self.filename}'
        )

    def odom_callback(self, msg):

        current_time = self.get_clock().now().nanoseconds / 1e9

        velocity = msg.twist.twist.linear.x

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        if self.previous_time is not None:

            dt = current_time - self.previous_time

            if dt > 0:
                self.actual_acceleration = (
                    velocity - self.previous_velocity
                ) / dt

        self.actual_velocity = velocity
        self.previous_velocity = velocity
        self.previous_time = current_time

    def control(self):

        current_time = self.get_clock().now().nanoseconds / 1e9
        t = current_time - self.start_time

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if t < 3.0:
            commanded_velocity = 0.0

        elif t < 8.0:
            commanded_velocity = 0.20

        elif t < 11.0:
            commanded_velocity = 0.0

        else:
            self.file.close()

            self.get_logger().info(
                f'Test complete. Data saved to {self.filename}'
            )

            rclpy.shutdown()
            return

        msg.twist.linear.x = commanded_velocity
        msg.twist.angular.z = 0.0

        self.publisher.publish(msg)

        self.csv_writer.writerow([
            t,
            commanded_velocity,
            self.actual_velocity,
            self.actual_acceleration,
            self.robot_x,
            self.robot_y
        ])

        self.file.flush()


def main(args=None):

    rclpy.init(args=args)

    node = VelocityTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
