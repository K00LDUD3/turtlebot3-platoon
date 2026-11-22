#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class LidarFilter(Node):

    def __init__(self):
        super().__init__('lidar_filter')
        self.set_parameters([
        rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )])

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.distance_publisher = self.create_publisher(
            Float32,
            '/filtered_distance',
            10
        )

    def scan_callback(self, msg):

        filtered_ranges = []

        for i, distance in enumerate(msg.ranges):

            angle = msg.angle_min + i * msg.angle_increment

            if angle > math.pi:
                angle -= 2.0 * math.pi

            if -math.radians(15) <= angle <= math.radians(15):

                if math.isfinite(distance):
                    if msg.range_min <= distance <= msg.range_max:
                        filtered_ranges.append(distance)

        if filtered_ranges:

            minimum = min(filtered_ranges)

            distance_msg = Float32()
            distance_msg.data = minimum

            self.distance_publisher.publish(distance_msg)

            print(
                f'Rays: {len(filtered_ranges):2d} | '
                f'Min distance: {minimum:.3f} m'
            )


def main(args=None):

    rclpy.init(args=args)

    node = LidarFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
