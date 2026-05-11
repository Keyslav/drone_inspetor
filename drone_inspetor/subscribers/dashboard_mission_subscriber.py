from rclpy.node import Node
from drone_inspetor.signals.dashboard_signals import MissionSignals
from drone_inspetor_msgs.msg import MissionStateMSG
from drone_inspetor.ros_interfaces import Topics, create_subscription_from
from drone_inspetor.common.msg_utils import msg_to_dict


class DashboardMissionSubscriber:
    """
    Gerencia a assinatura de tópicos internos do Mission Node e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: MissionSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals

        self.mission_state_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.MISSION_STATE, self.mission_state_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.mission_state_sub.topic_name}")

    def mission_state_callback(self, msg: MissionStateMSG):
        """Converte MissionStateMSG para dict e emite sinal mission_state_updated."""
        self.signals.mission_state_updated.emit(msg_to_dict(msg))
