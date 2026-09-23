"""Adaptador de sensores ROS para observações métricas e temporais NED."""

import math
import time

from drone_inspetor.navigation.obstacles import ObstacleMap
from drone_inspetor.common.coordinates import body_flu_offset_to_ned
from drone_inspetor.ros_interfaces import Topics, create_subscription_from


class NavigationSensors:
    """Consome scans originais; republicar flags antigas não renova a validade."""

    def __init__(self, node, config):
        self.node, self.config = node, config
        self.map = ObstacleMap(config.vehicle_radius + config.obstacle_margin,
                               config.sensor_timeout)
        self._source_stamps = {}
        self._last_ros_time = None
        self.down_distance = None
        self.down_received = None
        self.down_position = None
        create_subscription_from(node, Topics.Externo.LIDAR_SCAN, self.lidar_callback)
        create_subscription_from(node, Topics.Externo.LIDAR_DOWN_SCAN, self.down_callback)
        create_subscription_from(node, Topics.Interno.DEPTH_SCAN, self.depth_callback)

    def _accept(self, source, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        now_ros = self.node.get_clock().now().nanoseconds / 1e9
        if self._last_ros_time is not None and now_ros < self._last_ros_time:
            self._source_stamps.clear()
            self.map.scans.clear()
            self.down_received = None
        self._last_ros_time = now_ros
        # Header zero é permitido para drivers sem relógio, mas não recebe crédito
        # de repetição quando há timestamp disponível. O timeout usa monotonic.
        if stamp:
            if not 0 <= now_ros - stamp <= self.config.sensor_timeout:
                return None
            if stamp <= self._source_stamps.get(source, -math.inf):
                return None
            self._source_stamps[source] = stamp
        return max(0., now_ros - stamp) if stamp else 0.

    def lidar_callback(self, msg):
        self._scan_callback('lidar', msg, math.radians(self.config.lidar_mount_yaw_deg),
                            self.config.lidar_offset_forward, self.config.lidar_offset_left)

    def depth_callback(self, msg):
        # depth_node já transforma ângulos ópticos para FLU e aplica montagem yaw.
        self._scan_callback('depth', msg, 0., self.config.depth_offset_forward,
                            self.config.depth_offset_left)

    def _scan_callback(self, source, msg, mount_yaw, forward, left):
        age = self._accept(source, msg)
        if age is None or not self.node.telemetry_fresh():
            return
        now_ros = self.node.get_clock().now().nanoseconds / 1e9
        pose = self.node.pose_history.at(now_ros - age, self.config.sensor_pose_max_skew)
        if pose is None:
            self.map.scans.pop(source, None)
            return
        pos, yaw = pose.position, pose.yaw
        offset_ned = body_flu_offset_to_ned((forward, left, 0.), yaw)
        origin = tuple(value + offset for value, offset in zip(pos, offset_ned))
        self.map.update(source, msg.ranges, msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, origin, yaw,
                        time.monotonic() - age, mount_yaw)

    def down_callback(self, msg):
        age = self._accept('down', msg)
        if age is None:
            return
        now_ros = self.node.get_clock().now().nanoseconds / 1e9
        pose = self.node.pose_history.at(now_ros - age, self.config.sensor_pose_max_skew)
        self.down_position = pose.position if pose is not None else None
        valid = [float(value) for value in msg.ranges
                 if math.isfinite(value) and msg.range_min <= value <= msg.range_max]
        self.down_distance = min(valid) if valid else None
        self.down_received = time.monotonic() - age

    def descent_clearance(self, now):
        if (self.down_received is None or self.down_distance is None or self.down_position is None
                or now - self.down_received > self.config.sensor_timeout):
            return 0.0
        position = self.node.state_px4.local_position
        descent_since_scan = max(0., position.z - self.down_position[2])
        return max(0., self.down_distance - descent_since_scan - self.config.down_clearance)
