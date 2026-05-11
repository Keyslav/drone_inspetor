# drone_node.py (nodes/drone_node/drone_node.py)
# =================================================================================================
# NÓ DE CONTROLE DE BAIXO NÍVEL (OS "MÚSCULOS")
# =================================================================================================
# RESPONSABILIDADE PRINCIPAL:
# Atuar como a única e exclusiva interface de comunicação com a controladora de voo PX4.
# Ele traduz comandos de alto nível (recebidos do mission_node) em mensagens específicas do protocolo
# PX4 e, inversamente, traduz a telemetria complexa do PX4 em mensagens de status simplificadas
# para o mission_node e outros nós. Este nó NÃO contém lógica de missão.
# =================================================================================================

import rclpy
from rclpy.impl.rcutils_logger import Throttle
from rclpy.node import Node
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import math # Para cálculos de distância e conversões de ângulo

# --- Mensagens PX4 (constantes e construtores usados em callbacks/timers) ---
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleStatus,
)

# --- Mensagens customizadas (construtores e type hints) ---
from drone_inspetor_msgs.msg import DroneStateMSG, ObstaclesMSG

from drone_inspetor.common.enums import DroneStateDescription
from drone_inspetor.common.log_colors import LogPrefix
from drone_inspetor.common.param_utils import load_param
from drone_inspetor.ros_interfaces import (
    Topics,
    create_publisher_from,
    create_subscription_from,
    make_action_server,
)
from drone_inspetor.nodes.drone_node.fsm.context import DroneFSMContext
from drone_inspetor.nodes.drone_node.fsm.machine import DroneFSM
from drone_inspetor.nodes.drone_node.trajectory import DroneTrajectoryMixin
from drone_inspetor.nodes.drone_node.trajectory_profile import TrajectoryProfile
from drone_inspetor.nodes.drone_node.action_server import DroneActionServerMixin
from drone_inspetor.nodes.drone_node.px4_commands import DronePX4CommandsMixin


class DroneNode(DroneTrajectoryMixin, DroneActionServerMixin, DronePX4CommandsMixin, Node):
    """
    Gerencia a comunicação bidirecional com o PX4 e executa as ações de baixo nível
    sob o comando do mission_node. Atua como a camada de abstração de hardware para o drone.

    ARQUITETURA DE COMUNICAÇÃO:
    - Recebe comandos de alto nível do mission_node via DroneCommand Action
    - Traduz esses comandos em mensagens PX4 específicas
    - Monitora telemetria do PX4 e publica status simplificados para o mission_node
    - Republica dados relevantes para o Dashboard
    """



    def __init__(self):
        # --- Inicialização do Nó ROS2 ---
        super().__init__("drone_node")
        self.get_logger().info("================ INICIALIZANDO DRONE NODE ==============")

        # ==================================================================
        # PARÂMETROS
        # ==================================================================
        # Perfil trapezoidal de velocidade para deslocamentos GOTO/RTL.
        # ad é tanto aceleração quanto desaceleração nominal; ao é desaceleração por obstáculo.
        self.param_cruise_velocity = load_param(self, "cruise_velocity", 3.0)
        self.param_travel_acceleration = load_param(self, "travel_acceleration", 1.0)
        self.param_obstacle_deceleration = load_param(self, "obstacle_deceleration", 2.0)
        self.param_arrival_position_tol = load_param(self, "arrival_position_tol", 0.2)
        self.param_arrival_velocity_tol = load_param(self, "arrival_velocity_tol", 0.1)

        # Instância única do gerador de perfil — (re)inicializada por start_segment() a cada
        # entrada nos estados de deslocamento (VOANDO_A_CAMINHO, RETORNANDO_A_CAMINHO, etc.).
        self.trajectory_profile = TrajectoryProfile(
            vc=self.param_cruise_velocity,
            ad=self.param_travel_acceleration,
            ao=self.param_obstacle_deceleration,
            arrival_tol=self.param_arrival_position_tol,
            arrival_v_tol=self.param_arrival_velocity_tol,
        )

        # Flag para controle de shutdown limpo
        self._is_drone_node_shutting_down = False

        # ==================================================================
        # SUBSCRIBERS (Ouvindo o PX4 e o Mission Node)
        # ==================================================================

        # --- Tópicos de Telemetria do PX4 (/fmu/out/...) ---
        # Estes tópicos fornecem informações sobre o estado atual do drone
        create_subscription_from(self, Topics.PX4.VEHICLE_STATUS, self.px4_vehicle_status_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_COMMAND_ACK, self.px4_command_ack_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_LOCAL_POSITION, self.px4_vehicle_local_position_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_GLOBAL_POSITION, self.px4_vehicle_global_position_callback)
        create_subscription_from(self, Topics.PX4.HOME_POSITION, self.px4_home_position_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_ATTITUDE, self.px4_vehicle_attitude_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_LAND_DETECTED, self.px4_land_detected_callback)
        create_subscription_from(self, Topics.PX4.BATTERY_STATUS, self.px4_battery_status_callback)

        # --- Subscribers de obstáculos (lidar 360°+down e depth frontal) ---
        # As duas fontes alimentam DroneObstacles, que faz OR entre elas.
        create_subscription_from(self, Topics.Interno.LIDAR_OBSTACLE_DETECTIONS, self.lidar_obstacles_callback)
        create_subscription_from(self, Topics.Interno.DEPTH_OBSTACLE_DETECTIONS, self.depth_obstacles_callback)

        # --- Action Server para comandos do Mission Node ---
        # Este é o canal principal de comunicação com o Mission Node (usando DroneCommand Action)
        # O ActionServer permite:
        # - Feedback durante execução de comandos
        # - Resultado final da ação
        # - Capacidade de cancelar ações em andamento
        self._action_callback_group = ReentrantCallbackGroup()
        self._action_server = make_action_server(
            self,
            Topics.Action.DRONE_COMMAND,
            execute_callback=self.execute_drone_command_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_callback_group,
        )

        # ==================================================================
        # PUBLISHERS (Agindo no PX4 e falando com o Mission Node/Dashboard)
        # ==================================================================

        # --- Tópicos de Comando para o PX4 (/fmu/in/...) ---
        self.px4_vehicle_command_pub = create_publisher_from(self, Topics.PX4.VEHICLE_COMMAND)
        self.px4_offboard_control_mode_pub = create_publisher_from(self, Topics.PX4.OFFBOARD_CONTROL_MODE)
        self.px4_trajectory_setpoint_pub = create_publisher_from(self, Topics.PX4.TRAJECTORY_SETPOINT)

        # --- Tópicos de Status para o Mission Node e Dashboard ---
        self.drone_state_pub = create_publisher_from(self, Topics.Interno.DRONE_STATE)
        self.battery_status_pub = create_publisher_from(self, Topics.Interno.DRONE_BATTERY_STATUS)

        # ==================================================================
        # MÁQUINA DE ESTADOS
        # ==================================================================
        
        # Inicializa o contexto de dados e a máquina de estados do drone
        self.drone_context = DroneFSMContext(self)
        self.drone_fsm = DroneFSM(self.drone_context, self)

        # Inicializa estado interno da camada de comandos PX4 (mixin)
        self.init_px4_commands_state()

        # ==================================================================
        # TIMERS (Executando ações periódicas)
        # ==================================================================

        # Timers separados (0.02s): OffboardControlMode e TrajectorySetpoint
        self.px4_offboard_control_mode_timer = self.create_timer(0.02, self.px4_publish_offboard_control_mode)
        self.px4_trajectory_setpoint_timer = self.create_timer(0.02, self.px4_publish_trajectory_setpoint)

        # Timer para atualização da máquina de estados (a cada 0.5 segundos)
        self.drone_state_timer = self.create_timer(0.5, self.update_and_publish_drone_state)

        self.get_logger().info("================ DRONE NODE PRONTO ================")

    # ==================================================================
    # SEÇÃO 1: TIMERS - Funções Principais (executadas periodicamente)
    # ==================================================================

    def update_and_publish_drone_state(self):
        """
        Atualiza a máquina de estados periodicamente (a cada 0.5s).
        Este método é chamado pelo timer drone_state_timer.
        """
        self.drone_fsm.tick()
        self.publish_drone_status()



    def px4_publish_trajectory_setpoint(self):
        """Publica TrajectorySetpoint a cada 0.1s (se habilitado)."""
        if self.drone_context.state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return
        
        # Validação: precisa ter posição local e global para enviar setpoints
        if self.drone_context.state_px4.global_position is None or self.drone_context.state_px4.local_position is None:
            self.get_logger().error(LogPrefix.px4_rx("Não é possível publicar setpoints Offboard: Posição local ou global desconhecida."))
            return

        # Decide qual tipo de setpoint enviar baseado em on_trajectory (controlada pelos comandos/transições)
        if self.drone_context.on_trajectory and self.drone_context.target_local_position is not None:
            trajectory_msg = self.create_moving_trajectory_setpoint()
            # Publica o setpoint
            self.px4_trajectory_setpoint_pub.publish(trajectory_msg)



    def px4_publish_offboard_control_mode(self):
        """
        Publica o modo de controle Offboard habilitando apenas posição.
        """
        if self.drone_context.state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return

        offboard_msg = OffboardControlMode()
        # position é setpoint primário; velocity/acceleration entram como feedforward,
        # tornando o perfil trapezoidal previsível para o controlador interno do PX4.
        offboard_msg.position = True
        offboard_msg.velocity = True
        offboard_msg.acceleration = True
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.px4_offboard_control_mode_pub.publish(offboard_msg)


    def publish_drone_status(self):
        """
        Publica o status completo do drone_node como mensagem ROS DroneState.
        Inclui todas as variáveis necessárias para o Mission Node.
        """
        # Cria mensagem DroneState
        msg = DroneStateMSG()
        
        # --- Estado e Flags ---
        msg.state = self.drone_context.state
        msg.state_name = self.drone_context.state.name
        msg.state_duration_sec = round((self.get_clock().now().nanoseconds / 1e9) - self.drone_context.state_entry_time, 2)
        msg.is_armed = self.drone_context.state_px4.is_armed
        msg.is_landed = self.drone_context.state_px4.is_landed
        msg.is_on_trajectory = self.drone_context.on_trajectory
        
        # --- Posição Corrente Local (NED) ---
        if self.drone_context.state_px4.local_position:
            msg.current_local_x = self.drone_context.state_px4.local_position.x
            msg.current_local_y = self.drone_context.state_px4.local_position.y
            msg.current_local_z = -self.drone_context.state_px4.local_position.z  # Z é negativo, então invertemos
        else:
            msg.current_local_x = 0.0
            msg.current_local_y = 0.0
            msg.current_local_z = 0.0
        
        # --- Posição Corrente Global (GPS) ---
        if self.drone_context.state_px4.global_position:
            msg.current_latitude = self.drone_context.state_px4.global_position.lat
            msg.current_longitude = self.drone_context.state_px4.global_position.lon
            msg.current_altitude = self.drone_context.state_px4.global_position.alt
        else:
            msg.current_latitude = 0.0
            msg.current_longitude = 0.0
            msg.current_altitude = 0.0
        
        # --- Orientação Corrente ---
        # Publica ambas versões: deg (0-360) e deg_normalized (-180/180)
        current_yaw_normalized = self.drone_context.state_px4.current_yaw_deg_normalized
        current_yaw_deg = current_yaw_normalized if current_yaw_normalized >= 0 else current_yaw_normalized + 360
        msg.current_yaw_deg = current_yaw_deg
        msg.current_yaw_deg_normalized = current_yaw_normalized
        msg.current_yaw_rad = self.drone_context.state_px4.current_yaw_rad
        
        # --- Posição HOME Global (GPS) ---
        if self.drone_context.state_px4.home_global_lat is not None:
            msg.home_global_lat = self.drone_context.state_px4.home_global_lat
            msg.home_global_lon = self.drone_context.state_px4.home_global_lon if self.drone_context.state_px4.home_global_lon else float('nan')
            msg.home_global_alt = self.drone_context.state_px4.home_global_alt if self.drone_context.state_px4.home_global_alt else float('nan')
        else:
            msg.home_global_lat = float('nan')
            msg.home_global_lon = float('nan')
            msg.home_global_alt = float('nan')
        
        # --- Posição HOME Local (NED) ---
        if self.drone_context.state_px4.home_local_position is not None:
            msg.home_local_x = self.drone_context.state_px4.home_local_position[0]
            msg.home_local_y = self.drone_context.state_px4.home_local_position[1]
            msg.home_local_z = -self.drone_context.state_px4.home_local_position[2]  # Z é negativo, então invertemos
        else:
            msg.home_local_x = float('nan')
            msg.home_local_y = float('nan')
            msg.home_local_z = float('nan')
        
        # --- Orientação HOME ---
        if self.drone_context.state_px4.home_yaw_deg is not None:
            msg.home_yaw_deg = self.drone_context.state_px4.home_yaw_deg
            msg.home_yaw_deg_normalized = self.drone_context.state_px4.home_yaw_deg_normalized if self.drone_context.state_px4.home_yaw_deg_normalized else float('nan')
            msg.home_yaw_rad = self.drone_context.state_px4.home_yaw_rad if self.drone_context.state_px4.home_yaw_rad else float('nan')
        else:
            msg.home_yaw_deg = float('nan')
            msg.home_yaw_deg_normalized = float('nan')
            msg.home_yaw_rad = float('nan')
        
        # --- Posição Alvo Local (NED) ---
        if self.drone_context.target_local_position is not None:
            msg.target_local_x = self.drone_context.target_local_position[0]
            msg.target_local_y = self.drone_context.target_local_position[1]
            msg.target_local_z = -self.drone_context.target_local_position[2]  # Z é negativo, então invertemos
        else:
            msg.target_local_x = float('nan')
            msg.target_local_y = float('nan')
            msg.target_local_z = float('nan')
        
        # --- Posição Alvo Global (GPS) ---
        if self.drone_context.target_latitude is not None:
            msg.target_lat = self.drone_context.target_latitude
            msg.target_lon = self.drone_context.target_longitude if self.drone_context.target_longitude else float('nan')
            msg.target_alt = self.drone_context.target_altitude if self.drone_context.target_altitude else float('nan')
        else:
            msg.target_lat = float('nan')
            msg.target_lon = float('nan')
            msg.target_alt = float('nan')
        
        # --- Orientação Alvo ---
        if self.drone_context.target_direction_yaw_deg is not None:
            msg.target_direction_yaw_deg = self.drone_context.target_direction_yaw_deg
            msg.target_direction_yaw_deg_normalized = self.drone_context.target_direction_yaw_deg_normalized if self.drone_context.target_direction_yaw_deg_normalized else float('nan')
            msg.target_direction_yaw_rad = self.drone_context.target_direction_yaw_rad if self.drone_context.target_direction_yaw_rad else float('nan')
        else:
            msg.target_direction_yaw_deg = float('nan')
            msg.target_direction_yaw_deg_normalized = float('nan')
            msg.target_direction_yaw_rad = float('nan')
        
        if self.drone_context.target_final_yaw_deg is not None:
            msg.target_final_yaw_deg = self.drone_context.target_final_yaw_deg
            msg.target_final_yaw_deg_normalized = self.drone_context.target_final_yaw_deg_normalized if self.drone_context.target_final_yaw_deg_normalized else float('nan')
            msg.target_final_yaw_rad = self.drone_context.target_final_yaw_rad if self.drone_context.target_final_yaw_rad else float('nan')
        else:
            msg.target_final_yaw_deg = float('nan')
            msg.target_final_yaw_deg_normalized = float('nan')
            msg.target_final_yaw_rad = float('nan')
        
        # --- Ponto de Foco (para GOTO com use_focus=True) ---
        if self.drone_context.focus_latitude is not None:
            msg.focus_lat = self.drone_context.focus_latitude
            msg.focus_lon = self.drone_context.focus_longitude if self.drone_context.focus_longitude else float('nan')
            msg.focus_yaw_deg = self.drone_context.focus_yaw_deg if self.drone_context.focus_yaw_deg is not None else float('nan')
            msg.focus_yaw_deg_normalized = self.drone_context.focus_yaw_deg_normalized if self.drone_context.focus_yaw_deg_normalized is not None else float('nan')
            msg.focus_yaw_rad = self.drone_context.focus_yaw_rad if self.drone_context.focus_yaw_rad is not None else float('nan')
        else:
            msg.focus_lat = float('nan')
            msg.focus_lon = float('nan')
            msg.focus_yaw_deg = float('nan')
            msg.focus_yaw_deg_normalized = float('nan')
            msg.focus_yaw_rad = float('nan')
        
        # --- Última Posição Estática (para hover estável) ---
        if self.drone_context.last_static_position is not None:
            msg.last_static_position_x = self.drone_context.last_static_position[0]
            msg.last_static_position_y = self.drone_context.last_static_position[1]
            msg.last_static_position_z = -self.drone_context.last_static_position[2]  # Z é negativo, então invertemos
        else:
            msg.last_static_position_x = float('nan')
            msg.last_static_position_y = float('nan')
            msg.last_static_position_z = float('nan')
        
        if self.drone_context.last_static_yaw_deg is not None:
            msg.last_static_yaw_deg = self.drone_context.last_static_yaw_deg
            msg.last_static_yaw_deg_normalized = self.drone_context.last_static_yaw_deg_normalized if self.drone_context.last_static_yaw_deg_normalized else float('nan')
            msg.last_static_yaw_rad = self.drone_context.last_static_yaw_rad if self.drone_context.last_static_yaw_rad else float('nan')
        else:
            msg.last_static_yaw_deg = float('nan')
            msg.last_static_yaw_deg_normalized = float('nan')
            msg.last_static_yaw_rad = float('nan')
        
        # --- Ponto de Desvio de Trajetória (quando há obstáculos) ---
        # Drone está em desvio quando há snapshot do destino original ativo
        # (sub-FSM *_OBSTACULO_* salvou o destino antes de partir para o desvio).
        # Nesse caso, target_local_position aponta ao desvio atual.
        if (
            self.drone_context.saved_target_local_position is not None
            and self.drone_context.target_local_position is not None
        ):
            msg.has_trajectory_adjusted = True
            msg.trajectory_adjusted_x = self.drone_context.target_local_position[0]
            msg.trajectory_adjusted_y = self.drone_context.target_local_position[1]
            msg.trajectory_adjusted_z = -self.drone_context.target_local_position[2]  # Z é negativo, então invertemos
        else:
            msg.has_trajectory_adjusted = False
            msg.trajectory_adjusted_x = float('nan')
            msg.trajectory_adjusted_y = float('nan')
            msg.trajectory_adjusted_z = float('nan')
        
        # --- Velocidade Corrente (m/s) ---
        msg.current_velocity_x = self.drone_context.state_px4.current_velocity_x
        msg.current_velocity_y = self.drone_context.state_px4.current_velocity_y
        msg.current_velocity_z = self.drone_context.state_px4.current_velocity_z
        
        # --- Aceleração Corrente (m/s²) ---
        msg.current_acceleration_x = self.drone_context.state_px4.current_acceleration_x
        msg.current_acceleration_y = self.drone_context.state_px4.current_acceleration_y
        msg.current_acceleration_z = self.drone_context.state_px4.current_acceleration_z
        

        # Publica mensagem
        self.drone_state_pub.publish(msg)
        


    # ==================================================================
    # SEÇÃO 3: CALLBACKS - Funções Principais (processando telemetria)
    # ==================================================================

    def px4_vehicle_status_callback(self, msg):
        """
        Processa a mensagem de status do veículo vinda do PX4.
        Esta é uma das mensagens mais importantes, pois contém informações sobre:
        - Estado de armamento (armado/desarmado)
        - Modo de voo atual (manual, auto, offboard, etc.)
        - Modos de Navegação (decolando, voando, pousando, etc.)
        - Flags de sistema e segurança
        """
        
        # ==================== MONITORAMENTO DE MOTORES ========================================
        # Detecta transição de DESARMADO para ARMADO e vice-versa
        
        was_armed = self.drone_context.state_px4.is_armed
        is_now_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
        if is_now_armed and not was_armed:
            self.get_logger().info(LogPrefix.px4_rx("Motores: ARMADOS"), throttle_duration_sec=2)
        elif not is_now_armed and was_armed:
            self.get_logger().info(LogPrefix.px4_rx("Motores: DESARMADOS"), throttle_duration_sec=2)
            
        # Atualiza o estado de armamento
        self.drone_context.state_px4.is_armed = is_now_armed
            
        # ==================== MONITORAMENTO DE ESTADOS DE NAVEGAÇÃO ===========================
        # Monitora todos os estados de navegação possíveis do PX4 para detectar eventos importantes
        
        # ESTADO OFFBOARD ------
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # O novo estado é OFFBOARD
            if self.drone_context.state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # Se o estado anterior não era OFFBOARD, significa que acabamos de entrar no modo OFFBOARD
                self.get_logger().info(LogPrefix.px4_rx("Modos de Navegação: OFFBOARD (Modo Offboard Ativado)"), throttle_duration_sec=2)
        else:
            # O novo estado NÃO é OFFBOARD
            if self.drone_context.state_px4.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # Se o estado anterior era OFFBOARD, significa que acabamos de sair do modo OFFBOARD
                self.get_logger().info(LogPrefix.px4_rx("Modos de Navegação: Modo Offboard Desativado"), throttle_duration_sec=2)

        # Atualiza o Modos de Navegação
        self.drone_context.state_px4.nav_state = msg.nav_state

    
    
    
    def px4_vehicle_local_position_callback(self, msg):
        """
        Processa a mensagem de posição local do drone.
        A posição local é relativa ao ponto de partida (home) e é dada em metros.
        Coordenadas: X (norte), Y (leste), Z (para baixo, negativo = para cima)
        Também captura velocidade (vx, vy, vz) e aceleração (ax, ay, az).
        """
        self.drone_context.state_px4.local_position = msg
        
        # Captura velocidade corrente (m/s)
        self.drone_context.state_px4.current_velocity_x = msg.vx if hasattr(msg, 'vx') else 0.0
        self.drone_context.state_px4.current_velocity_y = msg.vy if hasattr(msg, 'vy') else 0.0
        self.drone_context.state_px4.current_velocity_z = msg.vz if hasattr(msg, 'vz') else 0.0
        
        # Captura aceleração corrente (m/s²)
        self.drone_context.state_px4.current_acceleration_x = msg.ax if hasattr(msg, 'ax') else 0.0
        self.drone_context.state_px4.current_acceleration_y = msg.ay if hasattr(msg, 'ay') else 0.0
        self.drone_context.state_px4.current_acceleration_z = msg.az if hasattr(msg, 'az') else 0.0


    def lidar_obstacles_callback(self, msg: ObstaclesMSG):
        """
        Processa obstáculos detectados pelo LiDAR (cobertura 360° + abaixo).
        Atualiza o buffer da fonte 'lidar' em DroneObstacles.
        """
        self.drone_context.obstacles.update_from_lidar(msg)

    def depth_obstacles_callback(self, msg: ObstaclesMSG):
        """
        Processa obstáculos detectados pela câmera depth (cobertura frontal).
        Atualiza o buffer da fonte 'depth' em DroneObstacles. As propriedades
        agregadas em DroneObstacles fazem OR com o lidar.
        """
        self.drone_context.obstacles.update_from_depth(msg)

    def px4_vehicle_global_position_callback(self, msg):
        """
        Processa a mensagem de posição global (GPS) do drone.
        Contém latitude, longitude e altitude em coordenadas globais.
        """
        self.drone_context.state_px4.global_position = msg
        

    def px4_vehicle_attitude_callback(self, msg):
        """
        Processa a mensagem de atitude (orientação) do drone.
        A atitude é representada como um quaternião (q[0]=w, q[1]=x, q[2]=y, q[3]=z).
        """
        self.drone_context.state_px4.vehicle_attitude = msg
        
        # Calcula e publica o yaw atual em graus
        q_w = msg.q[0]
        q_x = msg.q[1]
        q_y = msg.q[2]
        q_z = msg.q[3]
        
        # Conversão de quaternião para ângulo de Euler (yaw)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        
        from drone_inspetor.common.math_utils import normalize_yaw_deg
        self.drone_context.state_px4.current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.drone_context.state_px4.current_yaw_deg = math.degrees(self.drone_context.state_px4.current_yaw_rad)
        self.drone_context.state_px4.current_yaw_deg_normalized = normalize_yaw_deg(self.drone_context.state_px4.current_yaw_deg)
        

    def px4_land_detected_callback(self, msg):
        """
        Processa a mensagem de detecção de pouso.
        Indica se o drone detectou que pousou (através de sensores de pressão, acelerômetros, etc.)
        """
        self.drone_context.state_px4.is_landed = msg.landed


    def px4_home_position_callback(self, msg):
        """
        Processa posição de home (HomePosition) do PX4.
        Atualiza as variáveis de referência HOME em DroneState para uso no RTL e conversões GPS.
        
        Estrutura da mensagem HomePosition:
        - lat, lon, alt: Posição global (GPS)
        - x, y, z: Posição local (NED)
        - yaw: Orientação em radianos
        - valid_alt, valid_hpos, valid_lpos: Flags de validade
        """
        self.drone_context.state_px4.home_position = msg  # Armazena a mensagem original
        
        # Extrai coordenadas globais (GPS)
        self.drone_context.state_px4.home_global_lat = msg.lat
        self.drone_context.state_px4.home_global_lon = msg.lon
        self.drone_context.state_px4.home_global_alt = msg.alt
        
        # Extrai posição local [x, y, z]
        self.drone_context.state_px4.home_local_position = [msg.x, msg.y, msg.z]
        
        # Extrai yaw diretamente (já em radianos)
        yaw_rad = msg.yaw
        yaw_deg = math.degrees(yaw_rad)
        
        # Normaliza para -180 a 180
        yaw_deg_normalized = yaw_deg
        if yaw_deg_normalized > 180:
            yaw_deg_normalized -= 360
        elif yaw_deg_normalized < -180:
            yaw_deg_normalized += 360
        
        # Converte para 0-360
        yaw_deg_0_360 = yaw_deg_normalized
        if yaw_deg_0_360 < 0:
            yaw_deg_0_360 += 360
        
        self.drone_context.state_px4.home_yaw_rad = yaw_rad
        self.drone_context.state_px4.home_yaw_deg = yaw_deg_0_360
        self.drone_context.state_px4.home_yaw_deg_normalized = yaw_deg_normalized
        
        self.get_logger().info(
            LogPrefix.px4_rx(
                f"HOME atualizado via PX4: "
                f"GPS=[{self.drone_context.state_px4.home_global_lat:.6f}, {self.drone_context.state_px4.home_global_lon:.6f}, {self.drone_context.state_px4.home_global_alt:.2f}m], "
                f"Local=[{self.drone_context.state_px4.home_local_position[0]:.2f}, {self.drone_context.state_px4.home_local_position[1]:.2f}, {self.drone_context.state_px4.home_local_position[2]:.2f}], "
                f"Yaw={self.drone_context.state_px4.home_yaw_deg:.1f}°"
            )
        )


    def px4_battery_status_callback(self, msg):
        """
        Processa status da bateria (BatteryStatus)."""

        self.drone_context.state_px4.battery_status = msg
        self.battery_status_pub.publish(msg)


    def px4_failsafe_flags_callback(self, msg):
        """
        Processa flags de failsafe (FailsafeFlags)."""
        pass




    def destroy_node(self):
        """
        Override do método destroy_node para garantir shutdown seguro.
        """
        self._is_drone_node_shutting_down = True
        self.get_logger().info("Encerrando drone_node... (Flag _is_drone_node_shutting_down=True)")
        super().destroy_node()




def main(args=None):
    """Função principal do nó."""
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    drone_node = DroneNode()
    
    # Usa MultiThreadedExecutor para permitir que múltiplos callbacks
    # executem em paralelo (necessário para o ActionServer com loops de feedback)
    executor = MultiThreadedExecutor()
    executor.add_node(drone_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, Exception):
        pass  # Ignora todas as exceções durante shutdown (inclui contexto inválido)
    finally:
        try:
            drone_node.destroy_node()
        except Exception:
            pass  # Ignora erros ao destruir o nó
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()




