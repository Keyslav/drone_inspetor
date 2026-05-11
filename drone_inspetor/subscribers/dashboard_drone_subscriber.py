from rclpy.node import Node
from drone_inspetor.signals.dashboard_signals import DroneSignals, MapaSignals
from drone_inspetor_msgs.msg import DroneStateMSG
from drone_inspetor.ros_interfaces import Topics, create_subscription_from
from drone_inspetor.common.msg_utils import msg_to_dict


class DashboardDroneSubscriber:
    """
    Gerencia a assinatura de tópicos INTERNOS de controle (posição, atitude) e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, control_signals: DroneSignals, mapa_signals: MapaSignals):
        self.DashboardNode = DashboardNode
        self.control_signals = control_signals
        self.mapa_signals = mapa_signals

        self.drone_state_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.DRONE_STATE, self.drone_state_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.drone_state_sub.topic_name}")

    def drone_state_callback(self, msg: DroneStateMSG):
        """Converte DroneStateMSG para dict e emite sinais para mapa e controle."""
        try:
            drone_state = msg_to_dict(msg)
            self.mapa_signals.drone_state_updated.emit(drone_state)
            self.control_signals.drone_state_updated.emit(drone_state)
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao processar estado do drone: {e}")
