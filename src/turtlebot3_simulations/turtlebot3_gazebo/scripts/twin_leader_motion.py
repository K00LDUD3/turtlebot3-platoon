#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped


class TwinLeaderMotion(Node):

    def __init__(self):
        super().__init__('twin_leader_motion')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        self.declare_parameter('cmd_topic', '/leader/cmd_vel')
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.20)
        self.declare_parameter('loop', True)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.loop = self.get_parameter('loop').value

        self.publisher = self.create_publisher(
            TwistStamped,
            self.cmd_topic,
            10
        )

        # duration_s, linear_velocity, angular_velocity
        self.segments = [
            (3.0, 0.0, 0.0),
            (4.0, self.linear_speed, 0.0),
            (6.0, self.linear_speed, self.angular_speed),
            (4.0, self.linear_speed, 0.0),
            (6.0, self.linear_speed, -self.angular_speed),
            (8.0, self.linear_speed, 0.0),
            (3.0, 0.0, 0.0),
        ]

        self.segment_index = 0
        self.segment_start_time = None
        self.last_print_time = 0.0
        self.finished = False

        self.timer = self.create_timer(
            0.02,
            self.update
        )

        self.get_logger().info(
            f'Leader motion publishing to {self.cmd_topic}'
        )

    def publish_velocity(self, linear_x, angular_z):

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()

        command.twist.linear.x = linear_x
        command.twist.angular.z = angular_z

        self.publisher.publish(command)

    def update(self):

        now = self.get_clock().now().nanoseconds / 1e9

        if now <= 0.0:
            return

        if self.finished:
            return

        if self.segment_start_time is None:
            self.segment_start_time = now

        if self.segment_index >= len(self.segments):

            if self.loop:
                self.segment_index = 0
                self.segment_start_time = now
            else:
                self.publish_velocity(0.0, 0.0)
                self.finished = True
                self.timer.cancel()
                self.get_logger().info('Leader motion complete.')
                return

        duration, linear_velocity, angular_velocity = (
            self.segments[self.segment_index]
        )

        elapsed = now - self.segment_start_time

        if elapsed >= duration:
            self.segment_index += 1
            self.segment_start_time += duration
            if self.segment_index >= len(self.segments):
                return
            duration, linear_velocity, angular_velocity = (
                self.segments[self.segment_index]
            )
            elapsed = now - self.segment_start_time

        self.publish_velocity(linear_velocity, angular_velocity)

        if now - self.last_print_time >= 1.0:

            self.last_print_time = now

            self.get_logger().info(
                f'segment={self.segment_index} | '
                f't={elapsed:5.2f}/{duration:4.1f}s | '
                f'v={linear_velocity:+.2f} m/s | '
                f'w={angular_velocity:+.2f} rad/s'
            )

    def stop_leader(self):

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()

        command.twist.linear.x = 0.0
        command.twist.angular.z = 0.0

        for _ in range(5):
            self.publisher.publish(command)

        self.get_logger().info('Leader stopped.')


def main(args=None):

    rclpy.init(args=args)
    node = TwinLeaderMotion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_leader()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
