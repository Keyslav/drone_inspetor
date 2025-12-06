from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
from drone_inspetor.signals.dashboard_signals import MapaSignals

class DashboardMapaSubscriber:
    """
    Gerencia a assinatura de tópicos internos do Mapa e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: MapaSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals

        # QoS para dados de sensores: VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber para posição atualizada no mapa (tópico interno do dashboard)
        self.mapa_position_sub = self.DashboardNode.create_subscription(
            PoseStamped,
            "/drone_inspetor/dashboard/mapa/position",
            self.mapa_position_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info("Inscrito no tópico: /drone_inspetor/dashboard/mapa/position")

        # Subscriber para atitude atualizada no mapa (tópico interno do dashboard)
        self.mapa_attitude_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/dashboard/mapa/attitude",
            self.mapa_attitude_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info("Inscrito no tópico: /drone_inspetor/dashboard/mapa/attitude")

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


