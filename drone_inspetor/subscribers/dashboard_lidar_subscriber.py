from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray
import json
from drone_inspetor.signals.dashboard_signals import LidarSignals

class DashboardLidarSubscriber:
    """
    Gerencia a assinatura de tópicos internos de LiDAR e emite sinais PyQt
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

        # Subscriber para vetor de pontos do LiDAR
        self.lidar_point_vector_sub = self.DashboardNode.create_subscription(
            Float32MultiArray,
            "/drone_inspetor/lidar_node/point_vector",
            self.lidar_point_vector_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.lidar_point_vector_sub.topic_name}")

        # (Outros subscribers permanecem os mesmos)
        """
        self.lidar_stats_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/lidar_node/statistics",
            self.lidar_statistics_callback,
            10
        )
        self.lidar_obstacles_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/lidar_node/obstacle_detections",
            self.obstacle_detections_callback,
            10
        )"""

    def lidar_point_vector_callback(self, msg: Float32MultiArray):
        """
        Callback para mensagens do vetor de pontos do LiDAR.

        CORREÇÃO: Converte os dados da mensagem ROS para uma lista Python
        antes de emitir o sinal. A GUI receberá uma lista pura, não um objeto de mensagem.
        """
        try:
            # A propriedade 'data' da mensagem Float32MultiArray é um array-like.
            # Convertemos para uma lista Python padrão.
            point_vector_list = list(msg.data)
            
            # Emite o sinal com a lista de floats já processada.
            self.signals.point_vector_received.emit(point_vector_list)
            
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao processar vetor de pontos LiDAR no subscriber: {e}")

    def lidar_statistics_callback(self, msg: String):
        """
        Callback para mensagens de estatísticas do LiDAR.
        Decodifica o JSON e emite o sinal statistics_received da subclasse Lidar.
        """
        try:
            statistics = json.loads(msg.data)
            self.signals.statistics_received.emit(statistics)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de estatísticas LiDAR: {e}")

    def obstacle_detections_callback(self, msg: String):
        """
        Callback para mensagens de detecções de obstáculos do LiDAR.
        Decodifica o JSON e emite o sinal obstacle_detections_received da subclasse Lidar.
        """
        try:
            detections = json.loads(msg.data)
            self.signals.obstacle_detections_received.emit(detections)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de detecções de obstáculos LiDAR: {e}")
