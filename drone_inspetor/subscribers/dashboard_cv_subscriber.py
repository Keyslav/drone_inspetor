from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
from drone_inspetor.signals.dashboard_signals import CVSignals

class DashboardCVSubscriber:
    """
    Gerencia a assinatura de tópicos de Visão Computacional e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: CVSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals

        # QoS para dados de sensores (imagens): VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber para imagem processada por CV
        self.cv_image_sub = self.DashboardNode.create_subscription(
            Image,
            "/drone_inspetor/interno/cv_node/image_processed",
            self.cv_image_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.cv_image_sub.topic_name}")

        # Subscriber para detecções de objetos
        self.cv_detections_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/interno/cv_node/object_detections",
            self.cv_detections_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.cv_detections_sub.topic_name}")

        # Subscriber para dados de análise
        self.cv_analysis_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/interno/cv_node/analysis_report",
            self.cv_analysis_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.cv_analysis_sub.topic_name}")

    def cv_image_callback(self, msg):
        """
        Callback para mensagens de imagem processada por Visão Computacional.
        Emite o sinal image_received da subclasse CV.
        """
        self.signals.image_received.emit(msg)

    def cv_detections_callback(self, msg):
        """
        Callback para mensagens de detecções de objetos de Visão Computacional.
        Emite o sinal JSON (str) detections_received da subclasse CV.
        # Decodifica o JSON e emite o sinal detections_received da subclasse CV.
        """
        try:
            # detections = json.loads(msg.data)
            # self.signals.detections_received.emit(detections)
            self.signals.detections_received.emit(msg.data)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de detecções CV: {e}")

    def cv_analysis_callback(self, msg):
        """
        Callback para mensagens de dados de análise de Visão Computacional.
        Decodifica o JSON e emite o sinal analysis_data_received da subclasse CV.
        """
        try:
            analysis_data = json.loads(msg.data)
            self.signals.analysis_data_received.emit(analysis_data)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de análise CV: {e}")


