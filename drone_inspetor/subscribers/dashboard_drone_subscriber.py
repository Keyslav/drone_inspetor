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
        Converte mensagem ROS para dict completo e emite sinal único para o mapa.
        
        Todos os campos de DroneStateMSG são incluídos no dict emitido.
        """
        try:
            # Converte todos os campos de DroneStateMSG para dict
            drone_state = {
                # Estado e Flags
                "state": msg.state,
                "state_name": msg.state_name,
                "state_duration_sec": msg.state_duration_sec,
                "is_armed": msg.is_armed,
                "is_landed": msg.is_landed,
                "is_on_trajectory": msg.is_on_trajectory,
                
                # Posição Corrente Local (NED)
                "current_local_x": msg.current_local_x,
                "current_local_y": msg.current_local_y,
                "current_local_z": msg.current_local_z,
                
                # Posição Corrente Global (GPS)
                "current_latitude": msg.current_latitude,
                "current_longitude": msg.current_longitude,
                "current_altitude": msg.current_altitude,
                
                # Orientação Corrente
                "current_yaw_deg": msg.current_yaw_deg,
                "current_yaw_deg_normalized": msg.current_yaw_deg_normalized,
                "current_yaw_rad": msg.current_yaw_rad,
                
                # Posição HOME Global (GPS)
                "home_global_lat": msg.home_global_lat,
                "home_global_lon": msg.home_global_lon,
                "home_global_alt": msg.home_global_alt,
                
                # Posição HOME Local (NED)
                "home_local_x": msg.home_local_x,
                "home_local_y": msg.home_local_y,
                "home_local_z": msg.home_local_z,
                
                # Orientação HOME
                "home_yaw_deg": msg.home_yaw_deg,
                "home_yaw_deg_normalized": msg.home_yaw_deg_normalized,
                "home_yaw_rad": msg.home_yaw_rad,
                
                # Posição Alvo Local (NED)
                "target_local_x": msg.target_local_x,
                "target_local_y": msg.target_local_y,
                "target_local_z": msg.target_local_z,
                
                # Posição Alvo Global (GPS)
                "target_lat": msg.target_lat,
                "target_lon": msg.target_lon,
                "target_alt": msg.target_alt,
                
                # Orientação Alvo
                "target_direction_yaw_deg": msg.target_direction_yaw_deg,
                "target_direction_yaw_deg_normalized": msg.target_direction_yaw_deg_normalized,
                "target_direction_yaw_rad": msg.target_direction_yaw_rad,
                "target_final_yaw_deg": msg.target_final_yaw_deg,
                "target_final_yaw_deg_normalized": msg.target_final_yaw_deg_normalized,
                "target_final_yaw_rad": msg.target_final_yaw_rad,
                
                # Ponto de Foco (para GOTO_FOCUS)
                "focus_lat": msg.focus_lat,
                "focus_lon": msg.focus_lon,
                "focus_yaw_deg": msg.focus_yaw_deg,
                "focus_yaw_deg_normalized": msg.focus_yaw_deg_normalized,
                "focus_yaw_rad": msg.focus_yaw_rad,
                
                # Última Posição Estática (para hover estável)
                "last_static_position_x": msg.last_static_position_x,
                "last_static_position_y": msg.last_static_position_y,
                "last_static_position_z": msg.last_static_position_z,
                "last_static_yaw_deg": msg.last_static_yaw_deg,
                "last_static_yaw_deg_normalized": msg.last_static_yaw_deg_normalized,
                "last_static_yaw_rad": msg.last_static_yaw_rad,
            }
            
            # Emite sinal único com estado completo do drone para mapa e controle
            self.mapa_signals.drone_state_updated.emit(drone_state)
            self.control_signals.drone_state_updated.emit(drone_state)
                
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao processar estado do drone: {e}")

