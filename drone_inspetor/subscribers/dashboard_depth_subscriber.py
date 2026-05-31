from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import json
from drone_inspetor.signals.dashboard_signals import DepthSignals
from cv_bridge import CvBridge

class DashboardDepthSubscriber:
    """
    Gerencia a assinatura de tópicos de profundidade e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: DepthSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals
        self.bridge = CvBridge()

        # QoS para dados de sensores (imagens): BEST_EFFORT + KEEP_LAST + depth=1
        # (sempre o frame mais recente; o pipeline de render da GUI descarta atrasados)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber para imagem de profundidade (visualização comprimida em JPEG)
        self.depth_image_sub = self.DashboardNode.create_subscription(
            CompressedImage,
            "/drone_inspetor/interno/depth_node/compressed",
            self.depth_image_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.depth_image_sub.topic_name}")

        # Subscriber para estatísticas de profundidade
        self.depth_stats_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/interno/depth_node/statistics",
            self.depth_statistics_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.depth_stats_sub.topic_name}")

        # Subscriber para alertas de proximidade
        self.depth_alerts_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/interno/depth_node/proximity_alerts",
            self.proximity_alert_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.depth_alerts_sub.topic_name}")

    def depth_image_callback(self, msg):
        """
        Callback para mensagens de imagem da câmera de profundidade.
        Decodifica a CompressedImage para OpenCV e emite o sinal image_received.
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.signals.image_received.emit(cv_image)
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao converter imagem de profundidade comprimida: {e}")

    def depth_statistics_callback(self, msg):
        """
        Callback para mensagens de estatísticas da câmera de profundidade.
        Decodifica o JSON e emite o sinal statistics_received da subclasse Depth.
        """
        try:
            statistics = json.loads(msg.data)
            self.signals.statistics_received.emit(statistics)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de estatísticas de profundidade: {e}")

    def proximity_alert_callback(self, msg):
        """
        Callback para mensagens de alertas de proximidade da câmera de profundidade.
        Decodifica o JSON e emite o sinal proximity_alert_received da subclasse Depth.
        """
        try:
            alerts = json.loads(msg.data)
            self.signals.proximity_alert_received.emit(alerts)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de alertas de proximidade: {e}")


