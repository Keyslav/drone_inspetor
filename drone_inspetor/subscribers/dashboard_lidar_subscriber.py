"""
dashboard_lidar_subscriber.py — Subscriber para dados do LiDAR no dashboard.
"""

from rclpy.node import Node
from std_msgs.msg import String
from drone_inspetor_msgs.msg import LidarMSG, ObstaclesMSG
import json
from drone_inspetor.signals.dashboard_signals import LidarSignals
from drone_inspetor.ros_interfaces import Topics, create_subscription_from
from drone_inspetor.common.msg_utils import msg_to_dict


class DashboardLidarSubscriber:
    """
    Gerencia a assinatura de tópicos de LiDAR e emite sinais PyQt
    com dados já convertidos para tipos nativos do Python.
    """

    def __init__(self, DashboardNode: Node, signals: LidarSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals

        self.lidar_data_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.LIDAR_DATA, self.lidar_data_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.lidar_data_sub.topic_name}")

        self.lidar_obstacles_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.LIDAR_OBSTACLE_DETECTIONS, self.obstacle_detections_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.lidar_obstacles_sub.topic_name}")

    def lidar_data_callback(self, msg: LidarMSG):
        """Converte LidarMSG para dict e emite lidar_data_received."""
        try:
            self.signals.lidar_data_received.emit(msg_to_dict(msg))
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao processar dados LiDAR: {e}")

    def lidar_statistics_callback(self, msg: String):
        """Decodifica JSON de estatísticas e emite statistics_received."""
        try:
            self.signals.statistics_received.emit(json.loads(msg.data))
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de estatísticas LiDAR: {e}")

    def obstacle_detections_callback(self, msg: ObstaclesMSG):
        """Converte ObstaclesMSG para dict e emite obstacle_detections_received."""
        try:
            self.signals.obstacle_detections_received.emit(msg_to_dict(msg))
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao processar detecções de obstáculos LiDAR: {e}")
