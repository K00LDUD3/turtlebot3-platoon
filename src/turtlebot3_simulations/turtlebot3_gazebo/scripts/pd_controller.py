#!/usr/bin/env python3

import csv
import os
import time
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class PIDController(Node):

    def __init__(self):
        super().__init__('pid_controller')
# =========================
        # PID PARAMETERS
        # =========================
        self.set_parameters([
                rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True,
            ),
        ])
    
        self.target_distance = 0.30

        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.max_velocity = 0.20

        # Integral anti-windup limit
        self.integral_limit = 1.0

        # =========================
        # CONTROLLER STATE
        # =========================

        self.previous_error = None
        self.previous_time = None

        self.integral = 0.0

        self.latest_distance = None

        # =========================
        # ROBOT STATE
        # =========================

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.actual_linear_velocity = 0.0
        self.actual_angular_velocity = 0.0

        self.wall_x = 1.0
        self.wall_velocity = 0.0
        self.wall_timestamp = 0.0

        # =========================
        # ROS INTERFACES
        # =========================

        self.distance_subscription = self.create_subscription(
            Float32,
            '/filtered_distance',
            self.distance_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.wall_subscription = self.create_subscription(
            TwistStamped,
            '/wall_state',
            self.wall_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # =========================
        # TELEMETRY
        # =========================

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        log_directory = os.path.expanduser(
            '~/turtlebot3_ws/logs'
        )

        os.makedirs(log_directory, exist_ok=True)

        filename = time.strftime(
            'pid_run_%Y%m%d_%H%M%S.csv'
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
            'wall_time',
            'elapsed_time',
            'wall_timestamp',
            'wall_x',
            'wall_velocity',
            
            'distance',
            'target_distance',
            'error',

            'integral',
            'derivative',

            'p_output',
            'i_output',
            'd_output',
            'pid_output',

            'commanded_velocity',

            'robot_x',
            'robot_y',
            'robot_yaw',

            'actual_linear_velocity',
            'actual_angular_velocity'
        ])

        self.get_logger().info(
            f'Telemetry: {self.log_path}'
        )

        self.get_logger().info(
            f'PID gains: '
            f'Kp={self.kp}, '
            f'Ki={self.ki}, '
            f'Kd={self.kd}'
        )

    def wall_callback(self, msg):

        self.wall_x = msg.twist.linear.x
        self.wall_velocity = msg.twist.linear.y
    
        self.wall_timestamp = (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.actual_linear_velocity = msg.twist.twist.linear.x
        self.actual_angular_velocity = msg.twist.twist.angular.z 

    def distance_callback(self, msg):

        distance = msg.data

        current_time = self.get_clock().now().nanoseconds / 1e9

        error = distance - self.target_distance

        # First measurement establishes the controller state.
        if self.previous_error is None:

            self.previous_error = error
            self.previous_time = current_time
            self.latest_distance = distance

            return

        dt = current_time - self.previous_time

        if dt < 0.01:
            return

        # =========================
        # PID TERMS
        # =========================

        derivative = (
            error - self.previous_error
        ) / dt

        # Candidate integral
        new_integral = (
            self.integral + error * dt
        )

        # Anti-windup clamp
        new_integral = max(
            -self.integral_limit,
            min(
                self.integral_limit,
                new_integral
            )
        )

        # PID output before saturation
        p_output = self.kp * error
        i_output = self.ki * new_integral
        d_output = self.kd * derivative

        raw_output = (
            p_output
            + i_output
            + d_output
        )

        # =========================
        # OUTPUT SATURATION
        # =========================

        velocity = max(
            -self.max_velocity,
            min(
                self.max_velocity,
                raw_output
            )
        )

        # Basic anti-windup:
        # only accept the new integral if either
        # the output is not saturated or the integral
        # would move the output back toward the allowed range.
        saturated_high = raw_output > self.max_velocity
        saturated_low = raw_output < -self.max_velocity

        if not saturated_high and not saturated_low:

            self.integral = new_integral

        elif saturated_high and error < 0.0:

            self.integral = new_integral

        elif saturated_low and error > 0.0:

            self.integral = new_integral

        # Recalculate integral contribution using
        # the accepted integral state.
        i_output = self.ki * self.integral

        pid_output = (
            p_output
            + i_output
            + d_output
        )

        velocity = max(
            -self.max_velocity,
            min(
                self.max_velocity,
                pid_output
            )
        )

        # =========================
        # COMMAND
        # =========================

        command = TwistStamped()

        command.header.stamp = (
            self.get_clock().now().to_msg()
        )

        command.twist.linear.x = velocity
        command.twist.linear.y = 0.0
        command.twist.linear.z = 0.0

        command.twist.angular.x = 0.0
        command.twist.angular.y = 0.0
        command.twist.angular.z = 0.0

        self.cmd_publisher.publish(command)

        # =========================
        # TELEMETRY
        # =========================

        elapsed_time = (self.get_clock().now().nanoseconds / 1e9 - self.start_time)

        self.csv_writer.writerow([
            current_time,
            elapsed_time,
            self.wall_timestamp,
            self.wall_x,
            self.wall_velocity,
            
            distance,
            self.target_distance,
            error,

            self.integral,
            derivative,

            p_output,
            i_output,
            d_output,
            pid_output,

            velocity,

            self.robot_x,
            self.robot_y,
            self.robot_yaw,

            self.actual_linear_velocity,
            self.actual_angular_velocity
        ])

        self.log_file.flush()

        # =========================
        # LIVE OUTPUT
        # =========================

        print(
            f't={elapsed_time:7.2f}s | '
            f'd={distance:.3f}m | '
            f'e={error:+.3f}m | '
            f'P={p_output:+.3f} | '
            f'I={i_output:+.3f} | '
            f'D={d_output:+.3f} | '
            f'u={velocity:+.3f}m/s'
        )

        self.previous_error = error
        self.previous_time = current_time
        self.latest_distance = distance

    def stop_robot(self):

        command = TwistStamped()

        command.header.stamp = (
            self.get_clock().now().to_msg()
        )

        command.twist.linear.x = 0.0
        command.twist.angular.z = 0.0

        # Publish several zero commands so the robot
        # definitely receives the stop command.
        for _ in range(5):
            self.cmd_publisher.publish(command)

        self.log_file.flush()
        self.log_file.close()

        self.get_logger().info(
            'Robot stopped.'
        )

        self.get_logger().info(
            f'Telemetry saved to: {self.log_path}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PIDController()

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
