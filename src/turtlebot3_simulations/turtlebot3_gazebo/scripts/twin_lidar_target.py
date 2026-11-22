#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class TwinLidarTarget(Node):
    def __init__(self):
        super().__init__('twin_lidar_target')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.declare_parameter('scan_topic', '/follower/scan')
        self.declare_parameter('target_topic', '/follower/target')
        self.declare_parameter('sector_deg', 35.0)
        self.declare_parameter('cluster_band', 0.10)
        self.declare_parameter('track_edge', 'center')  # 'left', 'right', 'center'
        self.declare_parameter('edge_fraction', 0.30)    # Use 30% of the cluster for the edge

        self.scan_topic = self.get_parameter('scan_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.sector = math.radians(self.get_parameter('sector_deg').value)
        self.cluster_band = self.get_parameter('cluster_band').value
        self.track_edge = self.get_parameter('track_edge').value
        self.edge_fraction = self.get_parameter('edge_fraction').value

        self.subscription = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.publisher = self.create_publisher(PointStamped, self.target_topic, 10)
        self.get_logger().info(f'twin_lidar_target: tracking [{self.track_edge}] edge')

    def scan_callback(self, msg):
        selected = []
        for i, distance in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + i * msg.angle_increment)
            if abs(angle) > self.sector: continue
            if not math.isfinite(distance): continue
            if distance < msg.range_min or distance > msg.range_max: continue
            selected.append((angle, distance))

        output = PointStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = msg.header.frame_id

        if not selected:
            output.point.x = float('nan')
            self.publisher.publish(output)
            return

        minimum_distance = min(d for _, d in selected)
        cluster = [(a, d) for a, d in selected if d <= minimum_distance + self.cluster_band]
        
        if not cluster:
            output.point.x = float('nan')
            self.publisher.publish(output)
            return

        # Sort by angle (negative=right, positive=left)
        cluster.sort(key=lambda x: x[0])
        n = max(1, int(len(cluster) * self.edge_fraction))

        if self.track_edge == 'left':
            edge_cluster = cluster[-n:]   # Highest angles (Left)
        elif self.track_edge == 'right':
            edge_cluster = cluster[:n]    # Lowest angles (Right)
        else:
            edge_cluster = cluster        # Center

        sum_x = sum(math.cos(a) for a, _ in edge_cluster)
        sum_y = sum(math.sin(a) for a, _ in edge_cluster)
        target_angle = normalize_angle(math.atan2(sum_y, sum_x))

        output.point.x = minimum_distance
        output.point.y = target_angle
        output.point.z = 1.0
        self.publisher.publish(output)

        print(f'Rays: {len(cluster):2d} | d={minimum_distance:.3f} m | angle={math.degrees(target_angle):+6.1f} deg')

def main(args=None):
    rclpy.init(args=args)
    node = TwinLidarTarget()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
