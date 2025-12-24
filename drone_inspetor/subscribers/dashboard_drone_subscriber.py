from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from drone_inspetor.signals.dashboard_signals import DroneSignals, MapaSignals
from drone_inspetor_msgs.msg import DroneStateMSG

class DashboardDroneSubscriber:
    """
    Gerencia a assinatura de tópicos INTERNOS de controle (posição, atitude) e emite sinais PyQt.
    Este subscriber NÃO tem dependência direta com mensagens PX4.
    """
    def __init__(self, DashboardNode: Node, control_signals: DroneSignals, mapa_signals: MapaSignals):
        self.DashboardNode = DashboardNode
        self.control_signals = control_signals
        self.mapa_signals = mapa_signals

        # QoS para dados de sensores: TRANSIENT_LOCAL + BEST_EFFORT (estado atual disponível para novos subscribers)
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber para estado do drone (tópico interno do dashboard)
        self.drone_state_sub = self.DashboardNode.create_subscription(
            DroneStateMSG,
            "/drone_inspetor/interno/drone_node/drone_state",
            self.drone_state_callback,
            qos_status
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.drone_state_sub.topic_name}")

    def drone_state_callback(self, msg: DroneStateMSG):
        """
        Callback para mensagens de estado do drone.
        Converte mensagem ROS para dict e emite sinais para controle e mapa.
        """
        try:
            # Converte mensagem ROS para dict para compatibilidade com sinais PyQt
            position_data = {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude,
                "local_x": msg.local_x,
                "local_y": msg.local_y,
                "local_z": msg.local_z
            }
            
            attitude_data = {
                "yaw": msg.yaw_deg,
                "yaw_rad": msg.yaw_rad,
                "state": msg.state_name,
                "sub_state": msg.sub_state_name
            }
            
            self.control_signals.global_position_received.emit(position_data)
            self.mapa_signals.position_updated.emit(position_data)
            self.control_signals.attitude_received.emit(attitude_data)
            self.mapa_signals.attitude_updated.emit(attitude_data)
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao processar estado do drone: {e}")

