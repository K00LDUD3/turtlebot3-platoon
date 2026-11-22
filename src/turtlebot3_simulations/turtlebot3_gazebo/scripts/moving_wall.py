#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class MovingWall(Node):

    def __init__(self):
        super().__init__('moving_wall')

        self.set_parameters([
        rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
            )
        ])
        # Command the physical Gazebo wall
        self.wall_velocity_publisher = self.create_publisher(
            Float64,
            '/model/distance_wall/joint/wall_x_joint/cmd_vel',
            10
        )

        # Publish actual wall state for the PID controller
        self.wall_state_publisher = self.create_publisher(
            TwistStamped,
            '/wall_state',
            10
        )

        # Receive actual Gazebo joint state
        self.wall_joint_position = 0.0
        self.wall_joint_velocity = 0.0

        self.wall_joint_subscription = self.create_subscription(
            JointState,
            '/wall_joint_state',
            self.wall_joint_state_callback,
            10
        )

        # Wall motion limits
        self.x_min = 0.6
        self.x_max = 1.4

        self.max_velocity = 0.15
        self.max_acceleration = 0.15

        # Internal command state used only to generate the wall motion command
        self.position = 1.0
        self.velocity = 0.0
        self.direction = 1.0

        self.last_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

        self.timer = self.create_timer(
            0.02,
            self.update
        )

    def wall_joint_state_callback(self, msg):

        if 'wall_x_joint' not in msg.name:
            return

        index = msg.name.index('wall_x_joint')

        if index < len(msg.position):
            self.wall_joint_position = msg.position[index]

        if index < len(msg.velocity):
            self.wall_joint_velocity = msg.velocity[index]

    def update(self):

        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0:
            return

        # ---------------------------------------------------------
        # Generate the commanded wall motion
        # ---------------------------------------------------------

        distance_to_boundary = (
            self.x_max - self.position
            if self.direction > 0.0
            else self.position - self.x_min
        )

        stopping_distance = (
            self.velocity * self.velocity
            / (2.0 * self.max_acceleration)
        )

        if distance_to_boundary <= stopping_distance:
            acceleration = -self.direction * self.max_acceleration
        else:
            acceleration = self.direction * self.max_acceleration

        self.velocity += acceleration * dt

        self.velocity = max(
            -self.max_velocity,
            min(self.velocity, self.max_velocity)
        )

        self.position += self.velocity * dt

        if self.position >= self.x_max:
            self.position = self.x_max
            self.velocity = 0.0
            self.direction = -1.0

        elif self.position <= self.x_min:
            self.position = self.x_min
            self.velocity = 0.0
            self.direction = 1.0

        # Send commanded velocity to Gazebo
        velocity_msg = Float64()
        velocity_msg.data = self.velocity
        self.wall_velocity_publisher.publish(velocity_msg)

        # ---------------------------------------------------------
        # Publish ACTUAL Gazebo wall state
        # ---------------------------------------------------------

        # The joint position is displacement relative to the
        # wall's initial model pose of x = 1.0 m.
        actual_wall_x = 1.0 + self.wall_joint_position
        actual_wall_velocity = self.wall_joint_velocity

        state = TwistStamped()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'world'

        state.twist.linear.x = actual_wall_x
        state.twist.linear.y = actual_wall_velocity

        self.wall_state_publisher.publish(state)

        self.get_logger().info(
            f'command_x={self.position:.3f} m | '
            f'actual_x={actual_wall_x:.3f} m | '
            f'actual_v={actual_wall_velocity:.3f} m/s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = MovingWall()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
