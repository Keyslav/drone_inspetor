from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from drone_inspetor.signals.dashboard_signals import FSMSignals
from drone_inspetor_msgs.msg import FSMStateMSG

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
            FSMStateMSG,
            "/drone_inspetor/interno/fsm_node/fsm_state",
            self.fsm_state_callback,
            qos_status
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.fsm_state_sub.topic_name}")

    def fsm_state_callback(self, msg: FSMStateMSG):
        """
        Callback para mensagens de estado da FSM.
        Converte mensagem ROS para string e emite o sinal state_received da subclasse FSM.
        """
        # Converte mensagem ROS para string para compatibilidade com sinais PyQt
        state_str = msg.state_name
        self.DashboardNode.get_logger().info(f"Estado da FSM Recebido: {state_str}")
        self.signals.state_received.emit(state_str)

