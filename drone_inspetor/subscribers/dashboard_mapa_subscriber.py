from rclpy.node import Node
import json
from drone_inspetor.signals.dashboard_signals import MapaSignals
from drone_inspetor.ros_interfaces import Topics, create_subscription_from

class DashboardMapaSubscriber:
    """
    Gerencia a assinatura de tópicos internos do Mapa e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: MapaSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals

        # Subscriber para posição atualizada no mapa
        self.mapa_position_sub = create_subscription_from(
            self.DashboardNode, Topics.Dashboard.MAPA_POSITION, self.mapa_position_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.mapa_position_sub.topic_name}")

        # Subscriber para atitude atualizada no mapa
        self.mapa_attitude_sub = create_subscription_from(
            self.DashboardNode, Topics.Dashboard.MAPA_ATTITUDE, self.mapa_attitude_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.mapa_attitude_sub.topic_name}")

    def mapa_position_callback(self, msg):
        """
        Callback para mensagens de posição atualizada no mapa.
        Emite o sinal position_updated da subclasse Mapa.
        """
        data = {
            "lat": msg.pose.position.latitude, # Ajustar conforme a estrutura da sua mensagem PoseStamped
            "lon": msg.pose.position.longitude,
            "alt": msg.pose.position.altitude
        }
        self.signals.position_updated.emit(data)

    def mapa_attitude_callback(self, msg):
        """
        Callback para mensagens de atitude atualizada no mapa.
        Decodifica o JSON e emite o sinal attitude_updated da subclasse Mapa.
        """
        try:
            attitude_data = json.loads(msg.data)
            self.signals.attitude_updated.emit(attitude_data)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de atitude do mapa: {e}")


