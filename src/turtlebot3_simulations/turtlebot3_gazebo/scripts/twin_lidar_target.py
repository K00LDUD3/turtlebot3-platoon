#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TwinLidarTarget(Node):
    def __init__(self):
        super().__init__('twin_lidar_target')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True
            )
        ])

        self.declare_parameter('scan_topic', '/follower/scan')
        self.declare_parameter('odom_topic', '/follower/odom')
        self.declare_parameter('target_topic', '/follower/target')

        # Field-of-view and clustering
        self.declare_parameter('sector_deg', 45.0)
        self.declare_parameter('acquire_sector_deg', 15.0)
        self.declare_parameter('acquire_max_range', 1.0)
        self.declare_parameter('cluster_jump', 0.055)
        self.declare_parameter('min_cluster_rays', 3)
        self.declare_parameter('max_cluster_width', 0.22)

        # Target lock / calibration parameters
        self.declare_parameter('calibration_scans', 5)
        self.declare_parameter('track_gate', 0.14)
        self.declare_parameter('track_timeout', 1.0)
        self.declare_parameter('max_target_range', 2.0)

        self.declare_parameter('track_mode', 'center')  # 'center', 'left', 'right', 'outer'
        self.declare_parameter('edge_fraction', 0.30)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.target_topic = self.get_parameter('target_topic').value

        self.sector = math.radians(self.get_parameter('sector_deg').value)
        self.acquire_sector = math.radians(
            self.get_parameter('acquire_sector_deg').value
        )
        self.acquire_max_range = self.get_parameter('acquire_max_range').value
        self.cluster_jump = self.get_parameter('cluster_jump').value
        self.min_cluster_rays = self.get_parameter('min_cluster_rays').value
        self.max_cluster_width = self.get_parameter('max_cluster_width').value

        self.calibration_scans = self.get_parameter('calibration_scans').value
        self.track_gate = self.get_parameter('track_gate').value
        self.track_timeout = self.get_parameter('track_timeout').value
        self.max_target_range = self.get_parameter('max_target_range').value

        self.track_mode = self.get_parameter('track_mode').value
        self.edge_fraction = self.get_parameter('edge_fraction').value

        # Local follower odometry (zero inter-robot comms)
        self.odom_received = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Target lock state in follower's local odom frame
        self.locked = False
        self.calibration_samples = []
        self.target_wx = None
        self.target_wy = None
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.last_lock_time = None

        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.subscription = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.publisher = self.create_publisher(
            PointStamped, self.target_topic, 10
        )

        self.get_logger().info(
            f'twin_lidar_target: mode=[{self.track_mode}], '
            f'cluster_jump={self.cluster_jump:.3f}m, '
            f'track_gate={self.track_gate:.2f}m, '
            f'calibrating on first {self.calibration_scans} scans'
        )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def _segment_clusters(self, points):
        """Split angle-sorted (angle, distance) rays into distinct objects."""
        if not points:
            return []

        clusters = [[points[0]]]
        for a, d in points[1:]:
            pa, pd = clusters[-1][-1]
            x1, y1 = pd * math.cos(pa), pd * math.sin(pa)
            x2, y2 = d * math.cos(a), d * math.sin(a)
            if math.hypot(x2 - x1, y2 - y1) <= self.cluster_jump:
                clusters[-1].append((a, d))
            else:
                clusters.append([(a, d)])

        return [c for c in clusters if len(c) >= self.min_cluster_rays]

    def _cluster_features(self, cluster):
        min_d = min(d for _, d in cluster)
        sum_x = sum(math.cos(a) for a, _ in cluster)
        sum_y = sum(math.sin(a) for a, _ in cluster)
        theta = normalize_angle(math.atan2(sum_y, sum_x))

        # Cluster physical width (chord between first and last ray)
        a0, d0 = cluster[0]
        a1, d1 = cluster[-1]
        width = math.hypot(
            d1 * math.cos(a1) - d0 * math.cos(a0),
            d1 * math.sin(a1) - d0 * math.sin(a0),
        )

        # Position in follower's local odom frame
        if self.odom_received:
            world_angle = self.robot_yaw + theta
            wx = self.robot_x + min_d * math.cos(world_angle)
            wy = self.robot_y + min_d * math.sin(world_angle)
        else:
            wx = min_d * math.cos(theta)
            wy = min_d * math.sin(theta)

        return min_d, theta, width, wx, wy

    def _publish_invalid(self, frame_id):
        output = PointStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = frame_id
        output.point.x = float('nan')
        output.point.y = float('nan')
        output.point.z = 0.0
        self.publisher.publish(output)

    def scan_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        if not self.odom_received:
            return

        selected = []
        max_r = min(msg.range_max, self.max_target_range)
        for i, distance in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + i * msg.angle_increment)
            if abs(angle) > self.sector:
                continue
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > max_r:
                continue
            selected.append((angle, distance))

        if not selected:
            self._publish_invalid(msg.header.frame_id)
            return

        selected.sort(key=lambda x: x[0])
        clusters = self._segment_clusters(selected)
        candidates = []
        for cl in clusters:
            min_d, theta, width, wx, wy = self._cluster_features(cl)
            if width <= self.max_cluster_width:
                candidates.append((cl, min_d, theta, width, wx, wy))

        if not candidates:
            self._publish_invalid(msg.header.frame_id)
            return

        # ---------------------------------------------------------
        # PHASE 1: Calibration & Initial Target Lock
        # ---------------------------------------------------------
        if not self.locked:
            forward_candidates = [
                c for c in candidates
                if abs(c[2]) <= self.acquire_sector
                and c[1] <= self.acquire_max_range
            ]
            if not forward_candidates:
                self._publish_invalid(msg.header.frame_id)
                return

            chosen = min(forward_candidates, key=lambda c: c[1])
            _, min_d, theta_c, width, wx, wy = chosen
            self.calibration_samples.append((wx, wy))

            if len(self.calibration_samples) >= self.calibration_scans:
                self.target_wx = sum(p[0] for p in self.calibration_samples) / len(self.calibration_samples)
                self.target_wy = sum(p[1] for p in self.calibration_samples) / len(self.calibration_samples)
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.last_lock_time = now
                self.locked = True
                self.get_logger().info(
                    f'TARGET LOCKED at local odom ({self.target_wx:.3f}, {self.target_wy:.3f}) '
                    f'| d={min_d:.3f}m, width={width:.3f}m'
                )
            else:
                self.get_logger().info(
                    f'Calibrating target lock ({len(self.calibration_samples)}/{self.calibration_scans})...'
                )
            # Still publish during calibration so controller stays ready
            chosen_cluster = chosen[0]
        else:
            # ---------------------------------------------------------
            # PHASE 2: Locked Constant-Velocity Tracking in Local Odom
            # ---------------------------------------------------------
            dt = max(0.0, now - self.last_lock_time)
            if dt > self.track_timeout:
                # Target lost too long -> stop publishing invalid until re-seen
                # near last known predicted pose (do NOT blindly jump to pillars)
                self._publish_invalid(msg.header.frame_id)
                return

            pred_x = self.target_wx + self.target_vx * dt
            pred_y = self.target_wy + self.target_vy * dt

            best = None
            best_err = self.track_gate
            for c in candidates:
                err = math.hypot(c[4] - pred_x, c[5] - pred_y)
                if err < best_err:
                    best_err = err
                    best = c

            if best is None:
                self._publish_invalid(msg.header.frame_id)
                return

            chosen_cluster, min_d, theta_c, width, wx, wy = best

            # Smooth velocity estimate in odom frame
            if dt > 0.02:
                meas_vx = (wx - self.target_wx) / dt
                meas_vy = (wy - self.target_wy) / dt
                alpha_v = 0.4
                self.target_vx = (1.0 - alpha_v) * self.target_vx + alpha_v * meas_vx
                self.target_vy = (1.0 - alpha_v) * self.target_vy + alpha_v * meas_vy

            self.target_wx = wx
            self.target_wy = wy
            self.last_lock_time = now

        # Compute target angle according to track_mode
        n = max(1, int(len(chosen_cluster) * self.edge_fraction))
        if self.track_mode == 'outer':
            if theta_c > 0.05:
                edge_cluster = chosen_cluster[:n]
            elif theta_c < -0.05:
                edge_cluster = chosen_cluster[-n:]
            else:
                mid = len(chosen_cluster) // 2
                half_n = n // 2
                edge_cluster = chosen_cluster[mid - half_n : mid + half_n] or chosen_cluster
        elif self.track_mode == 'left':
            edge_cluster = chosen_cluster[-n:]
        elif self.track_mode == 'right':
            edge_cluster = chosen_cluster[:n]
        else:
            edge_cluster = chosen_cluster

        sum_x_edge = sum(math.cos(a) for a, _ in edge_cluster)
        sum_y_edge = sum(math.sin(a) for a, _ in edge_cluster)
        target_angle = normalize_angle(math.atan2(sum_y_edge, sum_x_edge))

        output = PointStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = msg.header.frame_id
        output.point.x = min_d
        output.point.y = target_angle
        output.point.z = 1.0
        self.publisher.publish(output)

        state = 'LOCK' if self.locked else 'CAL'
        print(
            f'[{state}] Rays: {len(chosen_cluster):2d} (cand={len(candidates)}) | '
            f'd={min_d:.3f} m | '
            f'ang={math.degrees(target_angle):+6.1f} deg | '
            f'w={width:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwinLidarTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
