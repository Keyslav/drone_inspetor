"""
dashboard_lidar_subscriber.py
=================================================================================================
Subscriber para dados do LiDAR no dashboard.

Recebe mensagens LidarMSG do lidar_node e emite sinais PyQt6 para a GUI.
=================================================================================================
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from drone_inspetor_msgs.msg import LidarMSG, LidarObstaclesMSG
import json
from drone_inspetor.signals.dashboard_signals import LidarSignals


class DashboardLidarSubscriber:
    """
    Gerencia a assinatura de tópicos de LiDAR e emite sinais PyQt
    com dados já convertidos para tipos nativos do Python.
    """
    
    def __init__(self, DashboardNode: Node, signals: LidarSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals

        # QoS para dados de sensores: VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber para dados consolidados do LiDAR (nova mensagem LidarMSG)
        self.lidar_data_sub = self.DashboardNode.create_subscription(
            LidarMSG,
            "/drone_inspetor/lidar_node/lidar_data",
            self.lidar_data_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.lidar_data_sub.topic_name}")

        # Outros subscribers para estatísticas e obstáculos (mantidos comentados)
        """
        self.lidar_stats_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/lidar_node/statistics",
            self.lidar_statistics_callback,
            10
        )
        """

        self.lidar_obstacles_sub = self.DashboardNode.create_subscription(
            LidarObstaclesMSG,
            "/drone_inspetor/interno/lidar_node/obstacle_detections",
            self.obstacle_detections_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.lidar_obstacles_sub.topic_name}")

    def lidar_data_callback(self, msg: LidarMSG):
        """
        Callback para mensagens LidarMSG.
        
        Converte a mensagem ROS2 para um dicionário Python e emite o sinal
        lidar_data_received para a GUI.
        """
        try:
            # Converte a mensagem para um dicionário
            lidar_data = {
                'point_vector': list(msg.point_vector),
                'ground_distance': float(msg.ground_distance)
            }
            
            # Emite o sinal com os dados consolidados
            self.signals.lidar_data_received.emit(lidar_data)
            
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao processar dados LiDAR no subscriber: {e}")

    def lidar_statistics_callback(self, msg: String):
        """
        Callback para mensagens de estatísticas do LiDAR.
        Decodifica o JSON e emite o sinal statistics_received.
        """
        try:
            statistics = json.loads(msg.data)
            self.signals.statistics_received.emit(statistics)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de estatísticas LiDAR: {e}")

    def obstacle_detections_callback(self, msg: LidarObstaclesMSG):
        """
        Callback para mensagens de detecções de obstáculos do LiDAR.
        Converte LidarObstaclesMSG para dicionário e emite o sinal obstacle_detections_received.
        """
        try:
            detections = {
                'have_obstacles_8m': msg.have_obstacles_8m,
                'have_obstacles_5m': msg.have_obstacles_5m,
                'have_obstacles_3m': msg.have_obstacles_3m,
                'have_obstacles_2m': msg.have_obstacles_2m,
                'have_obstacles_1m': msg.have_obstacles_1m,
                'have_obstacles_front_90': msg.have_obstacles_front_90,
                'have_obstacles_right_90': msg.have_obstacles_right_90,
                'have_obstacles_back_90': msg.have_obstacles_back_90,
                'have_obstacles_left_90': msg.have_obstacles_left_90,
                'have_obstacles_down_1m': msg.have_obstacles_down_1m,
                'have_obstacles_down_05m': msg.have_obstacles_down_05m,
            }
            self.signals.obstacle_detections_received.emit(detections)
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao processar detecções de obstáculos LiDAR: {e}")
