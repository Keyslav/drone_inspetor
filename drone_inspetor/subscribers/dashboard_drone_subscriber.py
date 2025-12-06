from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from drone_inspetor.signals.dashboard_signals import DroneSignals, MapaSignals
import json

class DashboardDroneSubscriber:
    """
    Gerencia a assinatura de tópicos INTERNOS de controle (posição, atitude) e emite sinais PyQt.
    Este subscriber NÃO tem dependência direta com mensagens PX4.
    """
    def __init__(self, DashboardNode: Node, control_signals: DroneSignals, mapa_signals: MapaSignals):
        self.DashboardNode = DashboardNode
        self.control_signals = control_signals
        self.mapa_signals = mapa_signals

        # QoS para dados de sensores: VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber para posição global (tópico interno do dashboard)
        self.dashboard_global_position_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/interno/drone_node/position",
            self.dashboard_global_position_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.dashboard_global_position_sub.topic_name}")

        # Subscriber para atitude (tópico interno do dashboard)
        self.dashboard_attitude_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/interno/drone_node/attitude",
            self.dashboard_attitude_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.dashboard_attitude_sub.topic_name}")

    def dashboard_global_position_callback(self, msg):
        """
        Callback para mensagens de posição global do drone (tópico interno do dashboard).
        Emite o sinal global_position_received para o controle e position_updated para o mapa.
        """
        try:
            data = json.loads(msg.data)
            self.control_signals.global_position_received.emit(data)
            self.mapa_signals.position_updated.emit(data)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de posição: {e}")

    def dashboard_attitude_callback(self, msg):
        """
        Callback para mensagens de atitude do drone (tópico interno do dashboard).
        Emite o sinal attitude_received para o controle e attitude_updated para o mapa.
        Assumindo que a mensagem é uma String JSON com roll, pitch, yaw, heading, speed.
        """
        try:
            attitude_data = json.loads(msg.data)
            self.control_signals.attitude_received.emit(attitude_data)
            self.mapa_signals.attitude_updated.emit(attitude_data)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de atitude: {e}")

