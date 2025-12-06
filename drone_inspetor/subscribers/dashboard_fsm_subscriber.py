from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from drone_inspetor.signals.dashboard_signals import FSMSignals

class DashboardFSMSubscriber:
    """
    Gerencia a assinatura de tópicos internos da FSM e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: FSMSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals

        # QoS para status do sistema: TRANSIENT_LOCAL + BEST_EFFORT (estado atual disponível para novos subscribers)
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber para o estado da FSM (tópico interno do dashboard)
        self.fsm_state_sub = self.DashboardNode.create_subscription(
            String,
            "/drone_inspetor/interno/fsm_node/state",
            self.fsm_state_callback,
            qos_status
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.fsm_state_sub.topic_name}")

    def fsm_state_callback(self, msg):
        """
        Callback para mensagens de estado da FSM.
        Emite o sinal state_received da subclasse FSM.
        """
        self.DashboardNode.get_logger().info(f"Estado da FSM Recebido: {msg.data}")
        self.signals.state_received.emit(msg.data)

