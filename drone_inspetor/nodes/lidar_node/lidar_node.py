"""Adaptador LiDAR para o dashboard; navegação utiliza os scans originais.

Flags legadas representam a última medição, sem cooldown e sem renovar a idade
por timers. Dados expirados são limpos uma vez, para não parecerem atuais na GUI.
"""

from dataclasses import dataclass, field
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from drone_inspetor.common.param_utils import load_param
from drone_inspetor.nodes.lidar_node.processing import (
    ground_distance, point_vector, scan_points, sector_flags,
)
from drone_inspetor.ros_interfaces import Topics, create_publisher_from, create_subscription_from
from drone_inspetor_msgs.msg import LidarMSG, ObstaclesMSG


@dataclass
class LidarData:
    """Somente apresentação: listas vazias/NaN representam dados desconhecidos."""

    point_vector: list = field(default_factory=list)
    ground_distance: float = math.nan

    def to_msg(self):
        message = LidarMSG()
        message.point_vector = self.point_vector
        message.ground_distance = self.ground_distance
        return message


class LidarNode(Node):
    """Limita a taxa do dashboard e substitui o snapshot a cada novo scan."""

    def __init__(self):
        super().__init__('lidar_node')
        data_rate = load_param(self, 'lidar_data_publish_rate', 5.0)
        obstacle_rate = load_param(self, 'obstacles_publish_rate', 5.0)
        self._timeout = load_param(self, 'sensor_timeout_seconds', 0.75)
        if not all(math.isfinite(value) and value > 0
                   for value in (data_rate, obstacle_rate, self._timeout)):
            raise ValueError('Taxas e timeout do LiDAR devem ser positivos')
        self.lidar_data = LidarData()
        self._horizontal_flags = sector_flags([], [])
        self._received = {'horizontal': None, 'down': None}
        self._stamps = {}
        self._generation = 0
        self._data_published = -1
        self._flags_published = -1
        self.laserscan_subscription = create_subscription_from(
            self, Topics.Externo.LIDAR_SCAN, self.laserscan_callback,
        )
        self.lidar_down_subscription = create_subscription_from(
            self, Topics.Externo.LIDAR_DOWN_SCAN, self.lidar_down_callback,
        )
        self.lidar_data_publisher = create_publisher_from(self, Topics.Interno.LIDAR_DATA)
        self.obstacles_publisher = create_publisher_from(self, Topics.Interno.LIDAR_OBSTACLE_DETECTIONS)
        self.lidar_data_timer = self.create_timer(1 / data_rate, self.publish_lidar_data)
        self.obstacles_timer = self.create_timer(1 / obstacle_rate, self.publish_obstacles)

    def _accept(self, source, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        now = time.monotonic()
        age = 0.0
        if stamp:
            now_ros = self.get_clock().now().nanoseconds / 1e9
            if self._stamps and now_ros < max(self._stamps.values()):
                self._stamps.clear()
                self._received = {'horizontal': None, 'down': None}
                self.lidar_data = LidarData()
                self._horizontal_flags = sector_flags([], [])
            age = now_ros - stamp
            if not 0 <= age <= self._timeout or stamp <= self._stamps.get(source, -math.inf):
                return False
            self._stamps[source] = stamp
        self._received[source] = now - age
        self._generation += 1
        return True

    def laserscan_callback(self, msg):
        if not self._accept('horizontal', msg):
            return
        try:
            ranges, angles = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                        msg.range_min, msg.range_max)
            self.lidar_data.point_vector = point_vector(ranges, angles)
            self._horizontal_flags = sector_flags(ranges, angles)
        except ValueError as exc:
            self.lidar_data.point_vector = []
            self._horizontal_flags = sector_flags([], [])
            self.get_logger().error(f'Scan LiDAR inválido: {exc}')

    def lidar_down_callback(self, msg):
        if not self._accept('down', msg):
            return
        try:
            self.lidar_data.ground_distance = ground_distance(msg.ranges, msg.range_min, msg.range_max)
        except ValueError as exc:
            self.lidar_data.ground_distance = math.nan
            self.get_logger().error(f'Scan inferior inválido: {exc}')

    def _expire(self):
        now = time.monotonic()
        for source, received in self._received.items():
            if received is None or now - received <= self._timeout:
                continue
            self._received[source] = None
            self._generation += 1
            if source == 'horizontal':
                self.lidar_data.point_vector = []
                self._horizontal_flags = sector_flags([], [])
            else:
                self.lidar_data.ground_distance = math.nan

    def _to_obstacles_msg(self):
        message = ObstaclesMSG()
        for name, value in self._horizontal_flags.items():
            setattr(message, name, value)
        distance = self.lidar_data.ground_distance
        message.have_obstacles_down_1m = math.isfinite(distance) and distance <= 1.0
        message.have_obstacles_down_05m = math.isfinite(distance) and distance <= 0.5
        return message

    def publish_lidar_data(self):
        self._expire()
        if self._data_published != self._generation:
            self.lidar_data_publisher.publish(self.lidar_data.to_msg())
            self._data_published = self._generation

    def publish_obstacles(self):
        self._expire()
        if self._flags_published != self._generation:
            self.obstacles_publisher.publish(self._to_obstacles_msg())
            self._flags_published = self._generation


def main(args=None):
    """Finaliza o nó preservando exceções operacionais inesperadas."""
    rclpy.init(args=args)
    node = None
    try:
        node = LidarNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
