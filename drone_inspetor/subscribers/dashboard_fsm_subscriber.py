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
        Converte mensagem ROS para dict e emite o sinal state_received com dados completos.
        
        Os campos emitidos correspondem aos campos de FSMStateMSG.msg:
        - state, state_name (estado atual)
        - on_mission, cancel_mission (flags de missão)
        - mission_name, tempo_de_permanencia (missão atual)
        - takeoff_altitude (parâmetros)
        - ponto_de_inspecao_indice_atual, ponto_de_inspecao_tempo_de_chegada (waypoints)
        """
        # Converte mensagem ROS para dict para compatibilidade com sinais PyQt
        state_data = {
            "state": msg.state,
            "state_name": msg.state_name,
            "on_mission": msg.on_mission,
            "cancel_mission": msg.cancel_mission,
            "mission_name": msg.mission_name,
            "mission_folder_path": msg.mission_folder_path,
            "tempo_de_permanencia": msg.tempo_de_permanencia,
            "takeoff_altitude": msg.takeoff_altitude,
            "ponto_de_inspecao_indice_atual": msg.ponto_de_inspecao_indice_atual,
            "total_pontos_de_inspecao": msg.total_pontos_de_inspecao,
            "ponto_de_inspecao_tempo_de_chegada": msg.ponto_de_inspecao_tempo_de_chegada,
            "objeto_alvo": msg.objeto_alvo,
            "tipos_anomalia": list(msg.tipos_anomalia)
        }
        self.signals.fsm_state_updated.emit(state_data)

