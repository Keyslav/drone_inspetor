from rclpy.node import Node
import json
from drone_inspetor.signals.dashboard_signals import DepthSignals
from drone_inspetor.ros_interfaces import Topics, create_subscription_from

class DashboardDepthSubscriber:
    """
    Gerencia a assinatura de tópicos de profundidade e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: DepthSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals

        # Subscriber para imagem de profundidade
        self.depth_image_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.DEPTH_IMAGE_PROCESSED, self.depth_image_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.depth_image_sub.topic_name}")

        # Subscriber para estatísticas de profundidade
        self.depth_stats_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.DEPTH_STATISTICS, self.depth_statistics_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.depth_stats_sub.topic_name}")

        # Subscriber para alertas de proximidade
        self.depth_alerts_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.DEPTH_PROXIMITY_ALERTS, self.proximity_alert_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.depth_alerts_sub.topic_name}")

    def depth_image_callback(self, msg):
        """
        Callback para mensagens de imagem da câmera de profundidade.
        Emite o sinal image_received da subclasse Depth.
        """
        self.signals.image_received.emit(msg)

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


