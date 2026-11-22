#!/usr/bin/env python3

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def saturate(value, limit):
    return max(-limit, min(limit, value))


class TwinFollowerController(Node):

    def __init__(self):
        super().__init__('twin_follower_controller')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        # =========================
        # PARAMETERS
        # =========================

        self.declare_parameter('target_topic', '/follower/target')
        self.declare_parameter('odom_topic', '/follower/odom')
        self.declare_parameter('cmd_topic', '/follower/cmd_vel')

        self.declare_parameter('target_distance', 0.30)

        self.declare_parameter('kp_linear', 4.0)
        self.declare_parameter('kd_linear', 0.4)

        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('kd_angular', 0.2)

        self.declare_parameter('angular_sign', 1.0)

        self.declare_parameter('max_linear_velocity', 0.20)
        self.declare_parameter('max_angular_velocity', 0.80)

        self.declare_parameter('target_timeout', 0.50)
        self.declare_parameter('max_steering_angle_deg', 60.0)

        self.target_topic = self.get_parameter('target_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        self.target_distance = self.get_parameter('target_distance').value

        self.kp_linear = self.get_parameter('kp_linear').value
        self.kd_linear = self.get_parameter('kd_linear').value

        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value

        self.angular_sign = self.get_parameter('angular_sign').value

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value

        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value

        self.target_timeout = self.get_parameter('target_timeout').value

        self.max_steering_angle = math.radians(
            self.get_parameter('max_steering_angle_deg').value
        )

        # =========================
        # STATE
        # =========================

        self.last_target_time = None

        self.previous_time = None
        self.previous_distance_error = None
        self.previous_angle_error = None

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.actual_linear_velocity = 0.0
        self.actual_angular_velocity = 0.0

        # =========================
        # ROS INTERFACES
        # =========================

        self.target_subscription = self.create_subscription(
            PointStamped,
            self.target_topic,
            self.target_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            self.cmd_topic,
            10
        )

        self.timeout_timer = self.create_timer(
            0.05,
            self.timeout_check
        )

        # =========================
        # TELEMETRY
        # =========================

        log_directory = os.path.expanduser(
            '~/turtlebot3_ws/logs'
        )

        os.makedirs(log_directory, exist_ok=True)

        filename = time.strftime(
            'twin_run_%Y%m%d_%H%M%S.csv'
        )

        self.log_path = os.path.join(
            log_directory,
            filename
        )

        self.log_file = open(
            self.log_path,
            'w',
            newline=''
        )

        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            'sim_time',
            'elapsed_time',

            'target_valid',
            'distance',
            'target_angle',

            'target_distance',
            'distance_error',
            'angle_error',

            'linear_p',
            'linear_d',
            'linear_raw',

            'angular_p',
            'angular_d',
            'angular_raw',

            'cmd_linear',
            'cmd_angular',

            'robot_x',
            'robot_y',
            'robot_yaw',

            'actual_linear_velocity',
            'actual_angular_velocity'
        ])

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            f'Telemetry: {self.log_path}'
        )

        self.get_logger().info(
            'Twin follower gains: '
            f'Kp_linear={self.kp_linear}, '
            f'Kd_linear={self.kd_linear}, '
            f'Kp_angular={self.kp_angular}, '
            f'Kd_angular={self.kd_angular}'
        )

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.actual_linear_velocity = msg.twist.twist.linear.x
        self.actual_angular_velocity = msg.twist.twist.angular.z

    def publish_zero(self):

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()

        command.twist.linear.x = 0.0
        command.twist.angular.z = 0.0

        self.cmd_publisher.publish(command)

    def timeout_check(self):

        if self.last_target_time is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_target_time > self.target_timeout:
            self.publish_zero()

            self.last_target_time = None
            self.previous_time = None
            self.previous_distance_error = None
            self.previous_angle_error = None

    def target_callback(self, msg):

        now = self.get_clock().now().nanoseconds / 1e9

        distance = msg.point.x
        target_angle = msg.point.y
        target_valid = msg.point.z > 0.5

        if (
            not target_valid
            or not math.isfinite(distance)
            or distance <= 0.0
            or not math.isfinite(target_angle)
        ):
            self.publish_zero()

            self.last_target_time = None
            self.previous_time = None
            self.previous_distance_error = None
            self.previous_angle_error = None
            return

        self.last_target_time = now

        distance_error = distance - self.target_distance
        angle_error = normalize_angle(target_angle)

        if self.previous_time is None:
            self.previous_time = now
            self.previous_distance_error = distance_error
            self.previous_angle_error = angle_error
            return

        dt = now - self.previous_time

        if dt < 0.01:
            return

        distance_derivative = (
            distance_error - self.previous_distance_error
        ) / dt

        angle_derivative = (
            normalize_angle(angle_error - self.previous_angle_error)
        ) / dt

        linear_p = self.kp_linear * distance_error
        linear_d = self.kd_linear * distance_derivative

        linear_raw = linear_p + linear_d

        linear_velocity = saturate(
            linear_raw,
            self.max_linear_velocity
        )

        angular_p = self.kp_angular * angle_error
        angular_d = self.kd_angular * angle_derivative

        angular_raw = angular_p + angular_d

        angular_velocity = self.angular_sign * saturate(
            angular_raw,
            self.max_angular_velocity
        )

        if abs(angle_error) > self.max_steering_angle:
            linear_velocity = 0.0
        else:
            linear_velocity *= max(0.0, math.cos(angle_error))

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()

        command.twist.linear.x = linear_velocity
        command.twist.angular.z = angular_velocity

        self.cmd_publisher.publish(command)

        elapsed_time = now - self.start_time

        self.csv_writer.writerow([
            now,
            elapsed_time,

            int(target_valid),
            distance,
            target_angle,

            self.target_distance,
            distance_error,
            angle_error,

            linear_p,
            linear_d,
            linear_raw,

            angular_p,
            angular_d,
            angular_raw,

            linear_velocity,
            angular_velocity,

            self.robot_x,
            self.robot_y,
            self.robot_yaw,

            self.actual_linear_velocity,
            self.actual_angular_velocity
        ])

        self.log_file.flush()

        print(
            f't={elapsed_time:7.2f}s | '
            f'd={distance:.3f}m | '
            f'ang={math.degrees(angle_error):+6.1f}deg | '
            f'v={linear_velocity:+.3f}m/s | '
            f'w={angular_velocity:+.3f}rad/s'
        )

        self.previous_time = now
        self.previous_distance_error = distance_error
        self.previous_angle_error = angle_error

    def stop_robot(self):

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()

        command.twist.linear.x = 0.0
        command.twist.angular.z = 0.0

        for _ in range(5):
            self.cmd_publisher.publish(command)

        self.log_file.flush()
        self.log_file.close()

        self.get_logger().info(
            'Follower stopped.'
        )

        self.get_logger().info(
            f'Telemetry saved to: {self.log_path}'
        )


def main(args=None):

    rclpy.init(args=args)
    node = TwinFollowerController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
