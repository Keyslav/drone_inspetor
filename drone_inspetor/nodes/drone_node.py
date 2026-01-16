# drone_node.py
# =================================================================================================
# NÓ DE CONTROLE DE BAIXO NÍVEL (OS "MÚSCULOS")
# =================================================================================================
# RESPONSABILIDADE PRINCIPAL:
# Atuar como a única e exclusiva interface de comunicação com a controladora de voo PX4.
# Ele traduz comandos de alto nível (recebidos da FSM) em mensagens específicas do protocolo
# PX4 e, inversamente, traduz a telemetria complexa do PX4 em mensagens de status simplificadas
# para a FSM e outros nós. Este nó NÃO contém lógica de missão.
# =================================================================================================

import rclpy
from rclpy.impl.rcutils_logger import Throttle
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import math # Para cálculos de distância e conversões de ângulo

# --- Importações de Mensagens PX4 ---
# Estas são as mensagens ROS2 geradas a partir dos arquivos .msg do PX4.
# Elas representam os dados que o drone envia (telemetria) e recebe (comandos).
from px4_msgs.msg import (
    ActuatorMotors,                 # Comandos para motores (uso avançado/direto)
    ActuatorServos,                 # Comandos para servos (uso avançado/direto)
    ArmingCheckReply,               # Resposta a uma requisição de checagem de armamento
    # AuxGlobalPosition,              # Posição global auxiliar (para sistemas externos)
    # ConfigControlSetpoints,         # Configuração de setpoints de controle
    # ConfigOverridesRequest,         # Requisição de sobrescrita de configurações
    DistanceSensor,                 # Dados de sensores de distância (ex: LiDAR, sonar)
    GotoSetpoint,                   # Setpoint para comando GOTO (navegação)
    # ManualControlInput,             # Entrada de controle manual (joystick, rádio)
    MessageFormatRequest,           # Requisição de formato de mensagem
    ModeCompleted,                  # Indicação de que um modo de voo foi completado
    ObstacleDistance,               # Distância para obstáculos (para prevenção de colisão)
    OffboardControlMode,            # Modo de controle Offboard (para controle externo de alto nível)
    OnboardComputerStatus,          # Status do computador de bordo
    RegisterExtComponentRequest,    # Requisição de registro de componente externo
    SensorOpticalFlow,              # Dados de sensor de fluxo óptico
    TelemetryStatus,                # Status da telemetria
    TrajectorySetpoint,             # Setpoint de trajetória (para controle de posição/velocidade)
    UnregisterExtComponent,         # Comando para desregistrar componente externo
    VehicleAttitudeSetpoint,        # Setpoint de atitude (roll, pitch, yaw)
    VehicleCommand,                 # Comando genérico para o veículo (armar, decolar, etc.)
    # VehicleCommandModeExecutor,     # Executor de modo de comando do veículo
    # VehicleMocapOdometry,           # Odometria de captura de movimento (motion capture)
    VehicleRatesSetpoint,           # Setpoint de taxas (velocidade angular de roll, pitch, yaw)
    VehicleThrustSetpoint,          # Setpoint de empuxo
    VehicleTorqueSetpoint,          # Setpoint de torque
    # VehicleVisualOdometry,          # Odometria visual
    AirspeedValidated,              # Velocidade do ar validada
    ArmingCheckRequest,             # Requisição de checagem de armamento
    BatteryStatus,                  # Status da bateria
    CollisionConstraints,           # Restrições de colisão
    EstimatorStatusFlags,           # Flags de status do estimador (EKF)
    FailsafeFlags,                  # Flags de failsafe
    HomePosition,                   # Posição de home (ponto de partida/retorno)
    ManualControlSetpoint,          # Setpoint de controle manual
    MessageFormatResponse,          # Resposta de formato de mensagem
    PositionSetpointTriplet,        # Tripla de setpoints de posição (atual, anterior, próximo)
    RegisterExtComponentReply,      # Resposta de registro de componente externo
    SensorCombined,                 # Dados combinados de sensores (IMU, etc.)
    TimesyncStatus,                 # Status de sincronização de tempo
    VehicleAttitude,                # Atitude atual do veículo (orientação)
    VehicleCommandAck,              # Confirmação de comando do veículo
    VehicleControlMode,             # Modo de controle atual do veículo
    VehicleGlobalPosition,          # Posição global atual (GPS)
    # VehicleGpsPosition,             # Posição GPS bruta
    VehicleLandDetected,            # Detecção de pouso
    VehicleLocalPosition,           # Posição local (relativa ao ponto de partida)
    VehicleOdometry,                # Odometria do veículo
    VehicleStatus,                  # Status geral do veículo (armado, modo de voo, etc.)
    VtolVehicleStatus               # Status específico de veículos VTOL
)

# --- Importações para comunicação com a FSM e o Dashboard ---
from std_msgs.msg import String

# --- Importação das mensagens ROS customizadas ---
from drone_inspetor_msgs.msg import DroneStateMSG
from drone_inspetor_msgs.action import DroneCommand

# --- Importação para Enums ---
from enum import IntEnum


class DroneNode(Node):
    """
    Gerencia a comunicação bidirecional com o PX4 e executa as ações de baixo nível
    sob o comando da FSM. Atua como a camada de abstração de hardware para o drone.
    
    ARQUITETURA DE COMUNICAÇÃO:
    - Recebe comandos de alto nível da FSM via tópico "/drone_inspetor/interno/fsm_node/drone_commands"
    - Traduz esses comandos em mensagens PX4 específicas
    - Monitora telemetria do PX4 e publica status simplificados para a FSM
    - Republica dados relevantes para o Dashboard
    """

    # Prefixos para destacar no terminal logs relacionados ao PX4
    # - RX (telemetria/mensagens vindas do PX4): ciano + negrito
    # - TX (comandos enviados ao PX4): magenta + negrito
    PX4_RX_PREFIX = "\033[36m\033[1mMENSAGEM PX4 - "
    PX4_TX_PREFIX = "\033[35m\033[1mCOMANDO PX4 - "

    # Prefixo para destacar no terminal logs relacionados ao FSM 
    # - Comandos vindos do FSM: azul + negrito
    FSM_RX_PREFIX = "\033[34m\033[1mCOMANDO FSM RECEBIDO - "

    # Tópicos PX4 (TX)
    PX4_TOPIC_VEHICLE_COMMAND = "/fmu/in/vehicle_command"
    
    def __init__(self):
        # --- Inicialização do Nó ROS2 ---
        super().__init__("drone_node")
        self.get_logger().info("================ INICIALIZANDO DRONE NODE ==============")

        # --- Configuração de QoS (Qualidade de Serviço) ---
        # O PX4 usa BEST_EFFORT para a maioria dos tópicos para reduzir latência
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Prioriza velocidade sobre garantia de entrega
            durability=DurabilityPolicy.TRANSIENT_LOCAL, # Mantém última mensagem para novos subscribers
            history=HistoryPolicy.KEEP_LAST,            # Mantém apenas a mensagem mais recente
            depth=1                                     # Buffer de 1 mensagem
        )

        # QoS para status críticos: TRANSIENT_LOCAL + BEST_EFFORT (estado atual disponível para novos subscribers)
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # ==================================================================
        # SUBSCRIBERS (Ouvindo o PX4 e a FSM)
        # ==================================================================

        # --- Tópicos de Telemetria do PX4 (/fmu/out/...) ---
        # Estes tópicos fornecem informações sobre o estado atual do drone
        
        # Status e controle do veículo
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.vehicle_status_callback, qos_profile)
        # self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.vehicle_control_mode_callback, qos_profile)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.command_ack_callback, qos_profile)
        # self.create_subscription(ModeCompleted, "/fmu/out/mode_completed", self.mode_completed_callback, qos_profile)
        
        # Posição e navegação
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_position_callback, qos_profile)
        self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_position_callback, qos_profile)
        # self.create_subscription(VehicleGpsPosition, "/fmu/out/vehicle_gps_position", self.vehicle_gps_position_callback, qos_profile)
        self.create_subscription(HomePosition, "/fmu/out/home_position", self.home_position_callback, qos_profile)
        
        # Atitude e orientação
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.vehicle_attitude_callback, qos_profile)
        # self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odometry_callback, qos_profile)
        
        # Detecção de eventos
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self.land_detected_callback, qos_profile)
        
        # Sensores e estimação
        # self.create_subscription(SensorCombined, "/fmu/out/sensor_combined", self.sensor_combined_callback, qos_profile)
        # self.create_subscription(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", self.estimator_status_flags_callback, qos_profile)
        # self.create_subscription(AirspeedValidated, "/fmu/out/airspeed_validated", self.airspeed_validated_callback, qos_profile)
        
        # Sistema e comunicação
        # self.create_subscription(TimesyncStatus, "/fmu/out/timesync_status", self.timesync_status_callback, qos_profile)
        self.create_subscription(BatteryStatus, "/fmu/out/battery_status", self.battery_status_callback, qos_profile)
        
        # Segurança e failsafe
        # self.create_subscription(FailsafeFlags, "/fmu/out/failsafe_flags", self.failsafe_flags_callback, qos_profile)
        # self.create_subscription(CollisionConstraints, "/fmu/out/collision_constraints", self.collision_constraints_callback, qos_profile)
        
        # Controle manual e setpoints
        # self.create_subscription(ManualControlSetpoint, "/fmu/out/manual_control_setpoint", self.manual_control_setpoint_callback, qos_profile)
        # self.create_subscription(PositionSetpointTriplet, "/fmu/out/position_setpoint_triplet", self.position_setpoint_triplet_callback, qos_profile)
        
        # VTOL específico
        # self.create_subscription(VtolVehicleStatus, "/fmu/out/vtol_vehicle_status", self.vtol_vehicle_status_callback, qos_profile)
        
        # Componentes externos e armamento
        # self.create_subscription(ArmingCheckRequest, "/fmu/out/arming_check_request", self.arming_check_request_callback, qos_profile)
        # self.create_subscription(MessageFormatResponse, "/fmu/out/message_format_response", self.message_format_response_callback, qos_profile)
        # self.create_subscription(RegisterExtComponentReply, "/fmu/out/register_ext_component_reply", self.register_ext_component_reply_callback, qos_profile)

        # --- Action Server para comandos da FSM ---
        # Este é o canal principal de comunicação com a FSM (usando DroneCommand Action)
        # O subscriber de tópico foi substituído por um ActionServer para:
        # - Feedback durante execução de comandos
        # - Resultado final da ação
        # - Capacidade de cancelar ações em andamento
        self._action_callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            DroneCommand,
            '/drone_inspetor/action/drone_command',
            execute_callback=self.execute_drone_command_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_callback_group
        )
        
        # Variáveis para controle de action em andamento
        self._current_goal_handle = None
        self._action_cancelled = False

        # ==================================================================
        # PUBLISHERS (Agindo no PX4 e falando com a FSM/Dashboard)
        # ==================================================================

        # --- Tópicos de Comando para o PX4 (/fmu/in/...) ---
        # Estes tópicos enviam comandos e setpoints para o PX4
        
        # Comandos principais
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        
        # Setpoints de trajetória e posição
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)
        # self.goto_setpoint_pub = self.create_publisher(GotoSetpoint, "/fmu/in/goto_setpoint", qos_profile)
        
        # Setpoints de atitude e taxas
        # self.vehicle_attitude_setpoint_pub = self.create_publisher(VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint", qos_profile)
        # self.vehicle_rates_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos_profile)
        
        # Setpoints de força e torque
        # self.vehicle_thrust_setpoint_pub = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", qos_profile)
        # self.vehicle_torque_setpoint_pub = self.create_publisher(VehicleTorqueSetpoint, "/fmu/in/vehicle_torque_setpoint", qos_profile)
        
        # Sensores externos
        # self.distance_sensor_pub = self.create_publisher(DistanceSensor, "/fmu/in/distance_sensor", qos_profile)
        # self.obstacle_distance_pub = self.create_publisher(ObstacleDistance, "/fmu/in/obstacle_distance", qos_profile)
        # self.sensor_optical_flow_pub = self.create_publisher(SensorOpticalFlow, "/fmu/in/sensor_optical_flow", qos_profile)
        
        # Odometria externa
        # self.vehicle_mocap_odometry_pub = self.create_publisher(VehicleMocapOdometry, "/fmu/in/vehicle_mocap_odometry", qos_profile)
        # self.vehicle_visual_odometry_pub = self.create_publisher(VehicleVisualOdometry, "/fmu/in/vehicle_visual_odometry", qos_profile)
        
        # Controle direto de atuadores (uso avançado)
        # self.actuator_motors_pub = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", qos_profile)
        # self.actuator_servos_pub = self.create_publisher(ActuatorServos, "/fmu/in/actuator_servos", qos_profile)
        
        # Controle manual e status
        # self.manual_control_input_pub = self.create_publisher(ManualControlInput, "/fmu/in/manual_control_input", qos_profile)
        # self.onboard_computer_status_pub = self.create_publisher(OnboardComputerStatus, "/fmu/in/onboard_computer_status", qos_profile)
        # self.telemetry_status_pub = self.create_publisher(TelemetryStatus, "/fmu/in/telemetry_status", qos_profile)
        
        # Armamento e componentes externos
        # self.arming_check_reply_pub = self.create_publisher(ArmingCheckReply, "/fmu/in/arming_check_reply", qos_profile)
        # self.register_ext_component_request_pub = self.create_publisher(RegisterExtComponentRequest, "/fmu/in/register_ext_component_request", qos_profile)
        # self.unregister_ext_component_pub = self.create_publisher(UnregisterExtComponent, "/fmu/in/unregister_ext_component", qos_profile)
        
        # Configuração e posição auxiliar
        # self.aux_global_position_pub = self.create_publisher(AuxGlobalPosition, "/fmu/in/aux_global_position", qos_profile)
        # self.config_control_setpoints_pub = self.create_publisher(ConfigControlSetpoints, "/fmu/in/config_control_setpoints", qos_profile)
        # self.config_overrides_request_pub = self.create_publisher(ConfigOverridesRequest, "/fmu/in/config_overrides_request", qos_profile)
        # self.message_format_request_pub = self.create_publisher(MessageFormatRequest, "/fmu/in/message_format_request", qos_profile)
        # self.vehicle_command_mode_executor_pub = self.create_publisher(VehicleCommandModeExecutor, "/fmu/in/vehicle_command_mode_executor", qos_profile)

        # --- Tópicos de Status para a FSM e Dashboard ---
        # Estes tópicos comunicam o estado do drone para outros nós do sistema
        
        self.drone_state_pub = self.create_publisher(DroneStateMSG, "/drone_inspetor/interno/drone_node/drone_state", qos_status)
        # self.global_position_pub = self.create_publisher(VehicleGlobalPosition, "/drone_inspetor/interno/drone_node/global_position", 10)
        #self.attitude_pub = self.create_publisher(VehicleAttitude, "/drone_inspetor/interno/drone_node/attitude", 10)
        # self.current_yaw_pub = self.create_publisher(String, "/drone_inspetor/interno/drone_node/current_yaw", 10)
        self.battery_status_pub = self.create_publisher(BatteryStatus, "/drone_inspetor/interno/drone_node/battery_status", qos_status)

        # ==================================================================
        # MÁQUINA DE ESTADOS
        # ==================================================================
        
        # Inicializa a máquina de estados do drone
        self.drone_state = DroneState(self)
        
        # Conjunto de comandos pendentes enviados pelo drone_node (para filtrar ACKs)
        self.pending_commands = set()

        # ==================================================================
        # TIMERS (Executando ações periódicas)
        # ==================================================================

        # Timers separados (0.02s): OffboardControlMode e TrajectorySetpoint
        self.offboard_control_mode_timer = self.create_timer(0.02, self.publish_offboard_control_mode)
        self.trajectory_setpoint_timer = self.create_timer(0.02, self.publish_trajectory_setpoint)

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
        self.drone_state.verifica_mudanca_de_estado_drone()
        self.publish_drone_status()

    def publish_trajectory_setpoint(self):
        """Publica TrajectorySetpoint a cada 0.1s (se habilitado)."""
        if self.drone_state.px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return
        
        # Validação: precisa ter posição local e global para enviar setpoints
        if self.drone_state.px4.global_position is None or self.drone_state.px4.local_position is None:
            self.get_logger().error(f"{self.PX4_RX_PREFIX}Não é possível publicar setpoints Offboard: Posição local desconhecida.")
            return

        # Decide qual tipo de setpoint enviar baseado em on_trajectory (controlada pelos comandos/transições)
        if self.drone_state.on_trajectory and self.drone_state.target_local_position is not None:
            trajectory_msg = self.create_moving_trajectory_setpoint()
            # Publica o setpoint
            self.trajectory_setpoint_pub.publish(trajectory_msg)
        #else:
            # Nenhum movimento: mantém posição atual (hover)
            #trajectory_msg = self.create_static_position_setpoint()
        
        



    def publish_offboard_control_mode(self):
        """
        Publica o modo de controle Offboard habilitando apenas posição.
        """
        if self.drone_state.px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return

        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False     # Desabilita feedforward de velocidade
        offboard_msg.acceleration = False # Desabilita feedforward de aceleração
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(offboard_msg)


    def publish_drone_status(self):
        """
        Publica o status completo do drone_node como mensagem ROS DroneState.
        Inclui todas as variáveis necessárias para a FSM.
        """
        # Cria mensagem DroneState
        msg = DroneStateMSG()
        
        # --- Estado e Flags ---
        msg.state = self.drone_state.state
        msg.state_name = self.drone_state.state.name
        msg.state_duration_sec = round(time.time() - self.drone_state.state_entry_time, 2)
        msg.is_armed = self.drone_state.px4.is_armed
        msg.is_landed = self.drone_state.px4.is_landed
        msg.is_on_trajectory = self.drone_state.on_trajectory
        
        # --- Posição Corrente Local (NED) ---
        if self.drone_state.px4.local_position:
            msg.current_local_x = self.drone_state.px4.local_position.x
            msg.current_local_y = self.drone_state.px4.local_position.y
            msg.current_local_z = -self.drone_state.px4.local_position.z  # Z é negativo, então invertemos
        else:
            msg.current_local_x = 0.0
            msg.current_local_y = 0.0
            msg.current_local_z = 0.0
        
        # --- Posição Corrente Global (GPS) ---
        if self.drone_state.px4.global_position:
            msg.current_latitude = self.drone_state.px4.global_position.lat
            msg.current_longitude = self.drone_state.px4.global_position.lon
            msg.current_altitude = self.drone_state.px4.global_position.alt
        else:
            msg.current_latitude = 0.0
            msg.current_longitude = 0.0
            msg.current_altitude = 0.0
        
        # --- Orientação Corrente ---
        # Publica ambas versões: deg (0-360) e deg_normalized (-180/180)
        current_yaw_normalized = self.drone_state.px4.current_yaw_deg_normalized
        current_yaw_deg = current_yaw_normalized if current_yaw_normalized >= 0 else current_yaw_normalized + 360
        msg.current_yaw_deg = current_yaw_deg
        msg.current_yaw_deg_normalized = current_yaw_normalized
        msg.current_yaw_rad = self.drone_state.px4.current_yaw_rad
        
        # --- Posição HOME Global (GPS) ---
        if self.drone_state.px4.home_global_lat is not None:
            msg.home_global_lat = self.drone_state.px4.home_global_lat
            msg.home_global_lon = self.drone_state.px4.home_global_lon if self.drone_state.px4.home_global_lon else float('nan')
            msg.home_global_alt = self.drone_state.px4.home_global_alt if self.drone_state.px4.home_global_alt else float('nan')
        else:
            msg.home_global_lat = float('nan')
            msg.home_global_lon = float('nan')
            msg.home_global_alt = float('nan')
        
        # --- Posição HOME Local (NED) ---
        if self.drone_state.px4.home_local_position is not None:
            msg.home_local_x = self.drone_state.px4.home_local_position[0]
            msg.home_local_y = self.drone_state.px4.home_local_position[1]
            msg.home_local_z = -self.drone_state.px4.home_local_position[2]  # Z é negativo, então invertemos
        else:
            msg.home_local_x = float('nan')
            msg.home_local_y = float('nan')
            msg.home_local_z = float('nan')
        
        # --- Orientação HOME ---
        if self.drone_state.px4.home_yaw_deg is not None:
            msg.home_yaw_deg = self.drone_state.px4.home_yaw_deg
            msg.home_yaw_deg_normalized = self.drone_state.px4.home_yaw_deg_normalized if self.drone_state.px4.home_yaw_deg_normalized else float('nan')
            msg.home_yaw_rad = self.drone_state.px4.home_yaw_rad if self.drone_state.px4.home_yaw_rad else float('nan')
        else:
            msg.home_yaw_deg = float('nan')
            msg.home_yaw_deg_normalized = float('nan')
            msg.home_yaw_rad = float('nan')
        
        # --- Posição Alvo Local (NED) ---
        if self.drone_state.target_local_position is not None:
            msg.target_local_x = self.drone_state.target_local_position[0]
            msg.target_local_y = self.drone_state.target_local_position[1]
            msg.target_local_z = -self.drone_state.target_local_position[2]  # Z é negativo, então invertemos
        else:
            msg.target_local_x = float('nan')
            msg.target_local_y = float('nan')
            msg.target_local_z = float('nan')
        
        # --- Posição Alvo Global (GPS) ---
        if self.drone_state.target_latitude is not None:
            msg.target_lat = self.drone_state.target_latitude
            msg.target_lon = self.drone_state.target_longitude if self.drone_state.target_longitude else float('nan')
            msg.target_alt = self.drone_state.target_altitude if self.drone_state.target_altitude else float('nan')
        else:
            msg.target_lat = float('nan')
            msg.target_lon = float('nan')
            msg.target_alt = float('nan')
        
        # --- Orientação Alvo ---
        if self.drone_state.target_direction_yaw_deg is not None:
            msg.target_direction_yaw_deg = self.drone_state.target_direction_yaw_deg
            msg.target_direction_yaw_deg_normalized = self.drone_state.target_direction_yaw_deg_normalized if self.drone_state.target_direction_yaw_deg_normalized else float('nan')
            msg.target_direction_yaw_rad = self.drone_state.target_direction_yaw_rad if self.drone_state.target_direction_yaw_rad else float('nan')
        else:
            msg.target_direction_yaw_deg = float('nan')
            msg.target_direction_yaw_deg_normalized = float('nan')
            msg.target_direction_yaw_rad = float('nan')
        
        if self.drone_state.target_final_yaw_deg is not None:
            msg.target_final_yaw_deg = self.drone_state.target_final_yaw_deg
            msg.target_final_yaw_deg_normalized = self.drone_state.target_final_yaw_deg_normalized if self.drone_state.target_final_yaw_deg_normalized else float('nan')
            msg.target_final_yaw_rad = self.drone_state.target_final_yaw_rad if self.drone_state.target_final_yaw_rad else float('nan')
        else:
            msg.target_final_yaw_deg = float('nan')
            msg.target_final_yaw_deg_normalized = float('nan')
            msg.target_final_yaw_rad = float('nan')
        
        # --- Ponto de Foco (para GOTO_FOCUS) ---
        if self.drone_state.focus_latitude is not None:
            msg.focus_lat = self.drone_state.focus_latitude
            msg.focus_lon = self.drone_state.focus_longitude if self.drone_state.focus_longitude else float('nan')
            msg.focus_yaw_deg = self.drone_state.focus_yaw_deg if self.drone_state.focus_yaw_deg is not None else float('nan')
            msg.focus_yaw_deg_normalized = self.drone_state.focus_yaw_deg_normalized if self.drone_state.focus_yaw_deg_normalized is not None else float('nan')
            msg.focus_yaw_rad = self.drone_state.focus_yaw_rad if self.drone_state.focus_yaw_rad is not None else float('nan')
        else:
            msg.focus_lat = float('nan')
            msg.focus_lon = float('nan')
            msg.focus_yaw_deg = float('nan')
            msg.focus_yaw_deg_normalized = float('nan')
            msg.focus_yaw_rad = float('nan')
        
        # --- Última Posição Estática (para hover estável) ---
        if self.drone_state.last_static_position is not None:
            msg.last_static_position_x = self.drone_state.last_static_position[0]
            msg.last_static_position_y = self.drone_state.last_static_position[1]
            msg.last_static_position_z = -self.drone_state.last_static_position[2]  # Z é negativo, então invertemos
        else:
            msg.last_static_position_x = float('nan')
            msg.last_static_position_y = float('nan')
            msg.last_static_position_z = float('nan')
        
        if self.drone_state.last_static_yaw_deg is not None:
            msg.last_static_yaw_deg = self.drone_state.last_static_yaw_deg
            msg.last_static_yaw_deg_normalized = self.drone_state.last_static_yaw_deg_normalized if self.drone_state.last_static_yaw_deg_normalized else float('nan')
            msg.last_static_yaw_rad = self.drone_state.last_static_yaw_rad if self.drone_state.last_static_yaw_rad else float('nan')
        else:
            msg.last_static_yaw_deg = float('nan')
            msg.last_static_yaw_deg_normalized = float('nan')
            msg.last_static_yaw_rad = float('nan')
        
        # Publica mensagem
        self.drone_state_pub.publish(msg)
        

    # ==================================================================
    # SEÇÃO 2: CÁLCULO DE TRAJETÓRIA - Funções Auxiliares
    # ==================================================================

    def create_static_position_setpoint(self):
        """
        Cria um setpoint de trajetória para manter a posição estável (hover).
        
        Usa a última posição estática armazenada (last_static_position) ao invés da posição atual
        para evitar instabilidade causada por flutuações de GPS ou vento.
        A last_static_position é atualizada ao finalizar cada comando de movimento.
        
        IMPORTANTE: Nunca envia local_position diretamente para o trajectory. Se não houver
        last_static_position, armazena a posição atual primeiro e depois usa esse valor.
        
        Returns:
            TrajectorySetpoint: Mensagem com última posição estática conhecida e yaw
        """
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Se não houver posição estática, armazena a posição atual primeiro
        if self.drone_state.last_static_position is None:
            if self.drone_state.px4.local_position is not None:
                self.drone_state.last_static_position = [
                    self.drone_state.px4.local_position.x,
                    self.drone_state.px4.local_position.y,
                    self.drone_state.px4.local_position.z
                ]
                self.get_logger().debug(
                    f"Posição estática inicializada: "
                    f"[{self.drone_state.last_static_position[0]:.2f}, "
                    f"{self.drone_state.last_static_position[1]:.2f}, "
                    f"{self.drone_state.last_static_position[2]:.2f}]"
                )
            else:
                # Posição local indisponível - usa origem como fallback seguro
                self.get_logger().warn("Posição local indisponível para criar static setpoint")
                trajectory_msg.position[0] = 0.0
                trajectory_msg.position[1] = 0.0
                trajectory_msg.position[2] = 0.0
                trajectory_msg.yaw = float('nan')
                return trajectory_msg
        
        # Se não houver yaw estático, armazena o yaw atual primeiro
        if self.drone_state.last_static_yaw_rad is None:
            # Armazena ambas versões (normalized = -180/180, deg = 0-360)
            self.drone_state.last_static_yaw_deg_normalized = self.drone_state.px4.current_yaw_deg_normalized
            self.drone_state.last_static_yaw_deg = self.drone_state.px4.current_yaw_deg_normalized
            if self.drone_state.last_static_yaw_deg < 0:
                self.drone_state.last_static_yaw_deg += 360
            self.drone_state.last_static_yaw_rad = self.drone_state.px4.current_yaw_rad
            self.get_logger().debug(
                f"Yaw estático inicializado: {self.drone_state.last_static_yaw_deg_normalized:.2f}°"
            )
        
        # Usa a posição estática armazenada para manter estabilidade
        trajectory_msg.position[0] = self.drone_state.last_static_position[0]
        trajectory_msg.position[1] = self.drone_state.last_static_position[1]
        trajectory_msg.position[2] = self.drone_state.last_static_position[2]
        
        # Usa o yaw estático armazenado para manter orientação durante hover
        trajectory_msg.yaw = self.drone_state.last_static_yaw_rad
        
        return trajectory_msg

    def create_moving_trajectory_setpoint(self):
        """
        Cria um setpoint de trajetória calculando o próximo ponto baseado no estado atual.
        Delega o cálculo para calculate_next_trajectory_by_state() que processa cada estado.
        
        GOTO: Rotação inicial → Movimento → Rotação final
        GOTO_FOCUS: Rotação para foco → Movimento apontando para foco → Estabilização
        RTL: Rotação para home → Movimento → Rotação para home_yaw → Pouso
        
        Returns:
            TrajectorySetpoint: Mensagem com setpoint completo
        """
        # Calcula posição e yaw na trajetória simplificada (yaw retornado em radianos)
        next_x, next_y, next_z, target_yaw_rad = self.calculate_next_trajectory_by_state()
        
        # Cria mensagem de trajetória
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Posição: próximo ponto a 1 metro em direção ao destino
        trajectory_msg.position[0] = next_x
        trajectory_msg.position[1] = next_y
        trajectory_msg.position[2] = next_z
        
        # Define o yaw alvo (já em radianos)
        if target_yaw_rad is not None:
            trajectory_msg.yaw = target_yaw_rad
        else:
            # Deixa o PX4 controlar o yaw por padrão
            trajectory_msg.yaw = float('nan')
        
        return trajectory_msg

    def _calculate_incremental_yaw(self, current_yaw_rad: float, target_yaw_rad: float, max_step_deg: float) -> float:
        """
        Calcula o próximo yaw incremental para rotação suave.
        
        Em vez de enviar o target_yaw diretamente, esta função calcula um passo
        intermediário de no máximo max_step_deg graus (ou a distância restante se menor).
        
        Args:
            current_yaw_rad: Yaw atual em radianos (-pi a pi)
            target_yaw_rad: Yaw alvo em radianos (-pi a pi)
            max_step_deg: Passo máximo em graus (ex: 45.0)
        
        Returns:
            float: Próximo yaw em radianos (-pi a pi)
        """
        # Converte para graus para facilitar cálculos
        current_deg = math.degrees(current_yaw_rad)
        target_deg = math.degrees(target_yaw_rad)
        
        # Calcula a diferença angular usando o caminho mais curto
        diff = target_deg - current_deg
        
        # Normaliza para -180 a 180 (caminho mais curto)
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        
        # Se a distância for menor que o passo máximo, vai direto para o alvo
        if abs(diff) <= max_step_deg:
            return target_yaw_rad
        
        # Caso contrário, avança apenas max_step_deg na direção correta
        if diff > 0:
            next_deg = current_deg + max_step_deg
        else:
            next_deg = current_deg - max_step_deg
        
        # Normaliza o resultado para -180 a 180
        while next_deg > 180:
            next_deg -= 360
        while next_deg < -180:
            next_deg += 360
        
        return math.radians(next_deg)

    def calculate_next_trajectory_by_state(self):
        """
        Calcula o próximo ponto da trajetória baseado na posição atual e no destino.
        Usa match/case para processar cada estado com verificações globais antes.
        
        ESTRUTURA:
        1. Verificações globais (posição local, posição alvo)
        2. Cálculos comuns (distância, direção)
        3. Match/case por estado para determinar setpoint
        
        Returns:
            Tupla (x, y, z, yaw_rad) onde:
            - x, y, z: Próxima posição em coordenadas locais (metros, frame NED)
            - yaw_rad: Yaw alvo em radianos (-pi a pi) ou None se não precisa controlar yaw
        """
        # === VERIFICAÇÕES GLOBAIS ===
        
        # Verifica se posição local está disponível
        if self.drone_state.px4.local_position is None:
            self.get_logger().warn("Trajetória: Posição local indisponível", throttle_duration_sec=2.0)
            return (0.0, 0.0, 0.0, None)
        
        # Verifica se posição alvo está definida
        if self.drone_state.target_local_position is None:
            self.get_logger().warn("Trajetória: Posição alvo não definida", throttle_duration_sec=2.0)
            # Mantém posição atual
            return (
                self.drone_state.px4.local_position.x,
                self.drone_state.px4.local_position.y,
                self.drone_state.px4.local_position.z,
                self.drone_state.px4.current_yaw_rad
            )
        
        # === CÁLCULOS COMUNS ===
        
        # Posição alvo
        target_x = self.drone_state.target_local_position[0]
        target_y = self.drone_state.target_local_position[1]
        target_z = self.drone_state.target_local_position[2]
        
        # Posição atual
        current_x = self.drone_state.px4.local_position.x
        current_y = self.drone_state.px4.local_position.y
        current_z = self.drone_state.px4.local_position.z
        
        # Posição atual
        origin_x = self.drone_state.origin_local_position[0]
        origin_y = self.drone_state.origin_local_position[1]
        origin_z = self.drone_state.origin_local_position[2]
        
        # Vetor de deslocamento e distância
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Vetor de distância para origem
        dxo = origin_x - current_x
        dyo = origin_y - current_y
        dzo = origin_z - current_z
        distance_to_origin = math.sqrt(dxo**2 + dyo**2 + dzo**2)
        
        # === MATCH/CASE POR ESTADO ===
        
        current_state = self.drone_state.state
        
        match current_state:
            
            # ============== ESTADOS DE DECOLAGEM ==============
            
            case DroneStateDescription.VOANDO_DECOLANDO:
                # Sobe até altitude alvo mantendo posição X,Y e yaw atual
                return (current_x, current_y, target_z, self.drone_state.px4.home_yaw_rad)
            
            # ============== ESTADOS DE TRAJETÓRIA GOTO ==============
            
            case DroneStateDescription.VOANDO_GIRANDO_INICIO:
                # Calcula yaw incremental (passo máximo de yaw_step_deg)
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_state.px4.current_yaw_rad,
                    self.drone_state.target_direction_yaw_rad,
                    self.drone_state.yaw_step_deg
                )
                # Mantém posição e gira incrementalmente para yaw de direção
                return (current_x, current_y, current_z, next_yaw_rad)
            
            case DroneStateDescription.VOANDO_A_CAMINHO:
                # Move em direção ao alvo
                # Determina yaw a usar: target_direction se disponível, senão mantém yaw atual
                if self.drone_state.target_direction_yaw_rad is not None:
                    yaw_to_use = self.drone_state.target_direction_yaw_rad
                elif self.drone_state.last_static_yaw_rad is not None:
                    yaw_to_use = self.drone_state.last_static_yaw_rad
                else:
                    yaw_to_use = self.drone_state.px4.current_yaw_rad
                
                if distance_to_target <= 1e-6:
                    return (target_x, target_y, target_z, yaw_to_use)
                
                # Normaliza e calcula próximo passo
                direction = [dx / distance_to_target, dy / distance_to_target, dz / distance_to_target]
                step_initial = max(1, distance_to_origin)
                step_dist = min(step_initial, self.drone_state.step_distance, distance_to_target)

                next_x = current_x + direction[0] * step_dist
                next_y = current_y + direction[1] * step_dist
                next_z = current_z + direction[2] * step_dist
                
                return (next_x, next_y, next_z, yaw_to_use)
            
            case DroneStateDescription.VOANDO_GIRANDO_COM_FOCO:
                # Mantém posição e gira para apontar ao focus
                focus_dx = self.drone_state.focus_local_position[0] - current_x
                focus_dy = self.drone_state.focus_local_position[1] - current_y
                focus_yaw_rad = math.atan2(focus_dy, focus_dx)
                focus_yaw_deg = math.degrees(focus_yaw_rad)
                if focus_yaw_deg > 180:
                    focus_yaw_deg -= 360
                    focus_yaw_rad = math.radians(focus_yaw_deg)
                elif focus_yaw_deg < -180:
                    focus_yaw_deg += 360
                    focus_yaw_rad = math.radians(focus_yaw_deg)
                # Armazena ambas versões (normalized = -180/180, deg = 0-360)
                self.drone_state.focus_yaw_deg_normalized = focus_yaw_deg
                focus_yaw_deg_0_360 = focus_yaw_deg if focus_yaw_deg >= 0 else focus_yaw_deg + 360
                self.drone_state.focus_yaw_deg = focus_yaw_deg_0_360
                self.drone_state.focus_yaw_rad = focus_yaw_rad
                # Calcula yaw incremental (passo máximo de yaw_step_deg)
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_state.px4.current_yaw_rad,
                    focus_yaw_rad,
                    self.drone_state.yaw_step_deg
                )
                return (current_x, current_y, current_z, next_yaw_rad)
            
            case DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO:
                # Move em direção ao alvo mantendo yaw apontando para o focus
                if distance_to_target <= 1e-6:
                    # Calcula yaw para o focus da posição alvo
                    focus_dx = self.drone_state.focus_local_position[0] - target_x
                    focus_dy = self.drone_state.focus_local_position[1] - target_y
                    focus_yaw_rad = math.atan2(focus_dy, focus_dx)
                    focus_yaw_deg = math.degrees(focus_yaw_rad)
                    if focus_yaw_deg > 180:
                        focus_yaw_deg -= 360
                        focus_yaw_rad = math.radians(focus_yaw_deg)
                    elif focus_yaw_deg < -180:
                        focus_yaw_deg += 360
                        focus_yaw_rad = math.radians(focus_yaw_deg)
                    # Armazena ambas versões (normalized = -180/180, deg = 0-360)
                    self.drone_state.focus_yaw_deg_normalized = focus_yaw_deg
                    focus_yaw_deg_0_360 = focus_yaw_deg if focus_yaw_deg >= 0 else focus_yaw_deg + 360
                    self.drone_state.focus_yaw_deg = focus_yaw_deg_0_360
                    self.drone_state.focus_yaw_rad = focus_yaw_rad
                    return (target_x, target_y, target_z, focus_yaw_rad)
                
                # Normaliza e calcula próximo passo
                direction = [dx / distance_to_target, dy / distance_to_target, dz / distance_to_target]
                step_initial = max(1, distance_to_origin)
                step_dist = min(step_initial, self.drone_state.step_distance, distance_to_target)
                
                next_x = current_x + direction[0] * step_dist
                next_y = current_y + direction[1] * step_dist
                next_z = current_z + direction[2] * step_dist
                
                # Calcula yaw dinamicamente para apontar da posição ATUAL para o focus
                focus_dx = self.drone_state.focus_local_position[0] - current_x
                focus_dy = self.drone_state.focus_local_position[1] - current_y
                focus_yaw_rad = math.atan2(focus_dy, focus_dx)
                focus_yaw_deg = math.degrees(focus_yaw_rad)
                # Normaliza para -180 a 180
                if focus_yaw_deg > 180:
                    focus_yaw_deg -= 360
                    focus_yaw_rad = math.radians(focus_yaw_deg)
                elif focus_yaw_deg < -180:
                    focus_yaw_deg += 360
                    focus_yaw_rad = math.radians(focus_yaw_deg)
                
                # Armazena ambas versões (normalized = -180/180, deg = 0-360)
                self.drone_state.focus_yaw_deg_normalized = focus_yaw_deg
                focus_yaw_deg_0_360 = focus_yaw_deg if focus_yaw_deg >= 0 else focus_yaw_deg + 360
                self.drone_state.focus_yaw_deg = focus_yaw_deg_0_360
                self.drone_state.focus_yaw_rad = focus_yaw_rad
                return (next_x, next_y, next_z, focus_yaw_rad)
            
            case DroneStateDescription.VOANDO_GIRANDO_FIM:
                # Na posição alvo, gira incrementalmente para yaw final
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_state.px4.current_yaw_rad,
                    self.drone_state.target_final_yaw_rad,
                    self.drone_state.yaw_step_deg
                )
                # Usa last_static_position se disponível para hover estável durante giro
                if self.drone_state.last_static_position is not None:
                    return (
                        self.drone_state.last_static_position[0],
                        self.drone_state.last_static_position[1],
                        self.drone_state.last_static_position[2],
                        next_yaw_rad
                    )
                return (target_x, target_y, target_z, next_yaw_rad)
            
            # ============== ESTADOS RTL (Return To Launch) ==============
            
            case DroneStateDescription.RETORNANDO_GIRANDO_INICIO:
                # Mantém posição e gira incrementalmente para direção do HOME
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_state.px4.current_yaw_rad,
                    self.drone_state.target_direction_yaw_rad,
                    self.drone_state.yaw_step_deg
                )
                return (current_x, current_y, current_z, next_yaw_rad)
            
            case DroneStateDescription.RETORNANDO_A_CAMINHO:
                # Move em direção ao HOME
                if distance_to_target <= 1e-6:
                    return (target_x, target_y, target_z, self.drone_state.target_direction_yaw_rad)
                
                # Normaliza e calcula próximo passo
                direction = [dx / distance_to_target, dy / distance_to_target, dz / distance_to_target]
                step_initial = max(1, distance_to_origin)
                step_dist = min(step_initial, self.drone_state.step_distance, distance_to_target)
                
                next_x = current_x + direction[0] * step_dist
                next_y = current_y + direction[1] * step_dist
                next_z = current_z + direction[2] * step_dist
                
                return (next_x, next_y, next_z, self.drone_state.target_direction_yaw_rad)
            
            case DroneStateDescription.RETORNANDO_GIRANDO_FIM:
                # Na posição HOME, gira incrementalmente para yaw final (home_yaw)
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_state.px4.current_yaw_rad,
                    self.drone_state.px4.home_yaw_rad,
                    self.drone_state.yaw_step_deg
                )
                # Usa last_static_position se disponível para hover estável durante giro
                if self.drone_state.last_static_position is not None:
                    return (
                        self.drone_state.last_static_position[0],
                        self.drone_state.last_static_position[1],
                        self.drone_state.last_static_position[2],
                        next_yaw_rad
                    )
                return (target_x, target_y, target_z, next_yaw_rad)
            
            # ============== ESTADO POUSANDO ==============
            
            case DroneStateDescription.POUSANDO:
                # O PX4 controla o pouso via AUTO_LAND
                # Mantém posição atual (não deveria ser chamado neste estado)
                return (current_x, current_y, current_z, self.drone_state.px4.home_yaw_rad)
            
            # ============== ESTADO PADRÃO ==============
            
            case _:
                # Estado desconhecido ou não requer trajetória - mantém posição atual
                self.get_logger().debug(
                    f"Trajetória: Estado {current_state.name} não requer setpoint específico",
                    throttle_duration_sec=5.0
                )
                return (current_x, current_y, current_z, self.drone_state.last_static_yaw_rad)


    def global_to_local_position(self, target_lat, target_lon, target_alt):
        """
        Converte coordenadas globais (GPS) para coordenadas locais (NED).
        
        Usa a posição GPS do HOME (onde local = 0,0,0) armazenada quando o drone armou.
        Isso garante que a conversão seja consistente durante todo o voo.
        
        Args:
            target_lat: Latitude alvo (graus)
            target_lon: Longitude alvo (graus)
            target_alt: Altitude alvo (metros)
        
        Returns:
            Lista [x, y, z] com posição local em metros (coordenadas NED)
            None se não houver referência HOME disponível
        """
        # Verifica se a referência HOME foi armazenada (quando armou)
        if self.drone_state.px4.home_global_lat is None:
            self.get_logger().warn(
                "global_to_local_position: Referência HOME não disponível. "
                "O drone precisa armar primeiro para definir a origem.",
                throttle_duration_sec=5.0
            )
            return None
        
        # Calcula offset da posição HOME (origem do frame local) para a posição alvo
        offset = self.global_to_local_offset(
            self.drone_state.px4.home_global_lat,
            self.drone_state.px4.home_global_lon,
            self.drone_state.px4.home_global_alt,
            target_lat,
            target_lon,
            target_alt
        )
        
        # O offset É a posição local diretamente (pois o HOME é onde local = 0,0,0)
        # No frame NED: X=norte, Y=leste, Z=para baixo (negativo = para cima)
        local_x = offset[0]  # Norte
        local_y = offset[1]  # Leste
        local_z = -offset[2]  # Para cima (inverte porque Z é negativo para cima no NED)
        
        return [local_x, local_y, local_z]
    

    def global_to_local_offset(self, lat1, lon1, alt1, lat2, lon2, alt2):
        """
        Converte diferença entre duas coordenadas globais para offset em coordenadas locais (metros).
        
        Args:
            lat1, lon1, alt1: Primeira coordenada global (origem)
            lat2, lon2, alt2: Segunda coordenada global (destino)
        
        Returns:
            Lista [dx, dy, dz] com offset em metros (coordenadas locais NED)
        """
        # Conversão simplificada de coordenadas globais para offset local
        lat_diff = (lat2 - lat1) * 111000  # metros por grau de latitude
        lon_diff = (lon2 - lon1) * 111000 * math.cos(math.radians(lat1))  # metros por grau de longitude
        alt_diff = alt2 - alt1  # diferença de altitude em metros
        
        return [lat_diff, lon_diff, alt_diff]
    
    # ==================================================================
    # ACTION SERVER CALLBACKS (comunicação com FSM)
    # ==================================================================

    def goal_callback(self, goal_request):
        """
        Callback para decidir se aceita ou rejeita um goal de action.
        Valida se o comando pode ser executado no estado atual do drone.
        
        Args:
            goal_request: Request contendo o Goal do action
            
        Returns:
            GoalResponse.ACCEPT se o comando é válido, GoalResponse.REJECT caso contrário
        """
        command = goal_request.command
        self.get_logger().info(f"{self.FSM_RX_PREFIX} (DroneCommand Action): {command}")
        
        # Valida se o comando é permitido no estado atual
        can_execute, error_msg = self.drone_state.verifica_validade_do_comando(command)
        
        if can_execute:
            self.get_logger().info(f"Comando da FSM aceito: {command}")
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn(f"Comando da FSM rejeitado: {error_msg}")
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """
        Callback para tratar requisições de cancelamento.
        Aceita o cancelamento e executa o comando STOP para parar o drone.
        
        Args:
            goal_handle: Handle do goal a ser cancelado
            
        Returns:
            CancelResponse.ACCEPT para aceitar o cancelamento
        """
        self.get_logger().info("Cancelamento de action solicitado. Executando STOP...")
        self._action_cancelled = True
        self.stop()  # Função existente que para o drone
        return CancelResponse.ACCEPT

    def execute_drone_command_callback(self, goal_handle):
        """
        Callback principal que executa o comando.
        Publica feedback periódico durante a execução e retorna o resultado.
        
        COMANDOS SUPORTADOS (DroneCommand Action):
        - command="ARM": Arma os motores
        - command="DISARM": Desarma os motores
        - command="TAKEOFF", altitude=3.0: Decola até a altitude especificada
        - command="LAND": Pousa na posição atual
        - command="GOTO", lat=..., lon=..., alt=..., yaw=...: Move o drone para posição
        - command="GOTO_FOCUS", lat=..., lon=..., alt=..., focus_lat=..., focus_lon=...: Move mantendo yaw no foco
        - command="RTL": Retorna para casa
        - command="STOP": Para o drone no ar durante GOTO, GOTO_FOCUS ou RTL
        
        Args:
            goal_handle: Handle do goal em execução
            
        Returns:
            DroneCommand.Result com o resultado da execução
        """
        self._current_goal_handle = goal_handle
        self._action_cancelled = False
        
        # Extrai parâmetros do goal
        request = goal_handle.request
        command = request.command
        
        # Executa o comando (inicia o movimento/ação)
        success = self._execute_command(request)
        
        if not success:
            result = DroneCommand.Result()
            result.success = False
            result.message = f"Falha ao iniciar comando: {command}"
            result.final_state = int(self.drone_state.state)
            goal_handle.abort()
            self._current_goal_handle = None
            return result
        
        # Para todos os comandos, publica feedback até completar
        feedback_msg = DroneCommand.Feedback()
        
        # Timeouts por tipo de comando (segundos)
        command_timeouts = {
            "ARM": 10.0,
            "DISARM": 10.0,
            "STOP": 5.0,
            "TAKEOFF": 60.0,
            "LAND": 60.0,
            "GOTO": 120.0,
            "GOTO_FOCUS": 120.0,
            "RTL": 180.0,
        }
        timeout = command_timeouts.get(command, 60.0)
        start_time = time.time()
        
        # Loop de feedback até o comando completar, cancelar ou timeout
        while not self._is_command_complete(command):
            # Verifica timeout
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().warn(f"Timeout ({timeout}s) aguardando conclusão de {command}")
                result = DroneCommand.Result()
                result.success = False
                result.message = f"Timeout ({timeout}s) aguardando conclusão de {command}"
                result.final_state = int(self.drone_state.state)
                goal_handle.abort()
                self._current_goal_handle = None
                return result
            
            # Verifica cancelamento
            if goal_handle.is_cancel_requested or self._action_cancelled:
                self.get_logger().info(f"Action cancelada durante execução de {command}")
                result = DroneCommand.Result()
                result.success = False
                result.message = "Ação cancelada"
                result.final_state = int(self.drone_state.state)
                goal_handle.canceled()
                self._current_goal_handle = None
                return result
            
            # Atualiza e publica feedback
            current_state = self.drone_state.state
            feedback_msg.current_state = int(current_state)
            feedback_msg.state_name = current_state.name
            feedback_msg.distance_to_target = self._calculate_distance_to_target()
            feedback_msg.progress_percent = self._calculate_progress_percent(command)
            goal_handle.publish_feedback(feedback_msg)
            
            # Aguarda antes do próximo ciclo
            time.sleep(0.1)
        
        # Comando completado com sucesso
        result = DroneCommand.Result()
        result.success = True
        result.message = f"Comando {command} completado com sucesso"
        result.final_state = int(self.drone_state.state)
        goal_handle.succeed()
        self._current_goal_handle = None
        return result

    def _execute_command(self, request) -> bool:
        """
        Executa o comando especificado no request da action.
        
        Args:
            request: Request da action contendo comando e parâmetros
            
        Returns:
            bool: True se o comando foi iniciado com sucesso, False caso contrário
        """
        command = request.command
        
        try:
            match command:
                case "ARM":
                    self.arm()
                case "DISARM":
                    self.disarm()
                case "TAKEOFF":
                    # Extrai altitude do request (usa padrão se NaN)
                    alt = request.altitude if not math.isnan(request.altitude) else self.drone_state.takeoff_altitude
                    self.takeoff(alt)
                case "GOTO":
                    # Extrai coordenadas do request (NaN = não especificado)
                    lat = request.lat if not math.isnan(request.lat) else None
                    lon = request.lon if not math.isnan(request.lon) else None
                    alt = request.alt if not math.isnan(request.alt) else None
                    yaw = request.yaw if not math.isnan(request.yaw) else None
                    self.goto(lat=lat, lon=lon, alt=alt, yaw=yaw)
                case "GOTO_FOCUS":
                    # Extrai coordenadas de destino e focus do request
                    lat = request.lat if not math.isnan(request.lat) else None
                    lon = request.lon if not math.isnan(request.lon) else None
                    alt = request.alt if not math.isnan(request.alt) else None
                    focus_lat = request.focus_lat if not math.isnan(request.focus_lat) else None
                    focus_lon = request.focus_lon if not math.isnan(request.focus_lon) else None
                    self.goto_focus(lat=lat, lon=lon, alt=alt, focus_lat=focus_lat, focus_lon=focus_lon)
                case "LAND":
                    self.land()
                case "RTL":
                    self.rtl()
                case "STOP":
                    self.stop()
                case _:
                    self.get_logger().warn(f"Comando não reconhecido: {command}")
                    return False
            return True
        except Exception as e:
            self.get_logger().error(f"Erro ao executar comando {command}: {e}")
            return False

    def _is_command_complete(self, command: str) -> bool:
        """
        Verifica se o comando foi completado.
        
        A lógica depende do tipo de comando:
        - ARM: drone está armado (POUSADO_ARMADO)
        - TAKEOFF: drone está voando pronto (VOANDO_PRONTO)
        - GOTO/GOTO_FOCUS: drone está voando pronto E flag de requisição foi limpa
        - LAND: drone está pousado (POUSADO_ARMADO ou POUSADO_DESARMADO)
        - RTL: drone está pousado e desarmado (POUSADO_DESARMADO)
        
        Args:
            command: Comando que está sendo executado
            
        Returns:
            bool: True se o comando foi completado
        """
        # Importa o enum localmente para evitar dependência circular
        from drone_inspetor.nodes.drone_node import DroneStateDescription
        
        current_state = self.drone_state.state
        
        match command:
            case "ARM":
                return current_state == DroneStateDescription.POUSADO_ARMADO
            
            case "DISARM":
                return current_state == DroneStateDescription.POUSADO_DESARMADO
            
            case "STOP":
                return current_state in [
                    DroneStateDescription.VOANDO_PRONTO,
                    DroneStateDescription.POUSADO_ARMADO, 
                    DroneStateDescription.POUSADO_DESARMADO
                ]
            
            case "TAKEOFF":
                return current_state == DroneStateDescription.VOANDO_PRONTO
            
            case "GOTO":
                # GOTO completo quando:
                # 1. Estado é VOANDO_PRONTO (terminou trajetória)
                # 2. Flag de requisição é False (garante que FSM processou o início)
                # Como GOTO agora SEMPRE transita para outros estados (GIRANDO_INICIO...),
                # se estiver em PRONTO com flag False, significa que completou o ciclo.
                is_processed = not self.drone_state.command_goto_requested
                return current_state == DroneStateDescription.VOANDO_PRONTO and is_processed

            case "GOTO_FOCUS":
                is_processed = not self.drone_state.command_goto_focus_requested
                return current_state == DroneStateDescription.VOANDO_PRONTO and is_processed
            
            case "LAND":
                return current_state in [
                    DroneStateDescription.POUSADO_ARMADO,
                    DroneStateDescription.POUSADO_DESARMADO
                ]
            
            case "RTL":
                return current_state == DroneStateDescription.POUSADO_DESARMADO
            
            case _:
                # Para comandos desconhecidos, considera completo imediatamente
                return True

    def _calculate_distance_to_target(self) -> float:
        """
        Calcula a distância restante até o alvo em metros.
        
        Returns:
            float: Distância em metros, ou 0.0 se não há alvo definido
        """
        if self.drone_state.target_local_position is None:
            return 0.0
        
        if self.drone_state.px4.local_position is None:
            return 0.0
        
        dx = self.drone_state.target_local_position[0] - self.drone_state.px4.local_position.x
        dy = self.drone_state.target_local_position[1] - self.drone_state.px4.local_position.y
        dz = self.drone_state.target_local_position[2] - self.drone_state.px4.local_position.z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def _calculate_progress_percent(self, command: str) -> float:
        """
        Calcula o percentual de progresso estimado para o comando.
        
        Args:
            command: Comando em execução
            
        Returns:
            float: Percentual de progresso (0.0 a 100.0)
        """
        # Para comandos de trajetória, usa a distância
        if command in ["GOTO", "GOTO_FOCUS", "RTL"]:
            if self.drone_state.initial_distance_to_target is not None and self.drone_state.initial_distance_to_target > 0:
                current_distance = self._calculate_distance_to_target()
                progress = 1.0 - (current_distance / self.drone_state.initial_distance_to_target)
                return max(0.0, min(100.0, progress * 100.0))
        
        # Para outros comandos, retorna 0 ou 100 baseado no estado
        return 0.0


    # ==================================================================
    # SEÇÃO 3: CALLBACKS - Funções Principais (processando telemetria)
    # ==================================================================

    def vehicle_status_callback(self, msg):
        """
        Processa a mensagem de status do veículo vinda do PX4.
        Esta é uma das mensagens mais importantes, pois contém informações sobre:
        - Estado de armamento (armado/desarmado)
        - Modo de voo atual (manual, auto, offboard, etc.)
        - Modos de Navegação (decolando, voando, pousando, etc.)
        - Flags de sistema e segurança
        """
        
        # Detecta transição de DESARMADO para ARMADO
        was_armed = self.drone_state.px4.is_armed
        is_now_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
        # Atualiza o estado de armamento
        self.drone_state.px4.is_armed = is_now_armed
            
        # ==================== MONITORAMENTO DE ESTADOS DE NAVEGAÇÃO ====================
        # Monitora todos os estados de navegação possíveis do PX4 para detectar eventos importantes
        
        # ==================== ESTADO OFFBOARD ====================
        # Controle externo via ROS2 (usado pelo drone_node)
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Se o novo estado é OFFBOARD
            if self.drone_state.px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # Se o estado anterior não era OFFBOARD, significa que acabamos de entrar
                self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: OFFBOARD (Modo Offboard Ativado)", throttle_duration_sec=2)
        else:
            # Se o novo estado NÃO é OFFBOARD
            if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # Se o estado anterior era OFFBOARD, significa que saímos do modo Offboard
                self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: Modo Offboard Desativado", throttle_duration_sec=2)

        # Atualiza o Modos de Navegação
        self.drone_state.px4.nav_state = msg.nav_state
        
        return

        # ==================== ESTADOS MANUAIS ====================
        # Controle manual direto pelo piloto (sem assistência de estabilização)
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_MANUAL:
            # Modo manual: controle direto dos motores pelo piloto
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: MANUAL", throttle_duration_sec=2)
            pass
        
        # Controle manual com estabilização de atitude
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_STAB:
            # Modo de estabilização: mantém atitude nivelada, mas permite controle manual
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: STAB (Estabilização)", throttle_duration_sec=2)
            pass
        
        # Controle acrobático: permite manobras acrobáticas sem estabilização
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_ACRO:
            # Modo acrobático: controle direto das taxas de rotação
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: ACRO (Acrobático)", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADOS ASSISTIDOS ====================
        # Controle manual com assistência de altitude
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_ALTCTL:
            # Modo de controle de altitude: mantém altitude fixa, permite controle manual horizontal
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: ALTCTL (Controle de Altitude)", throttle_duration_sec=2)
            pass
        
        # Controle manual com assistência de posição
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
            # Modo de controle de posição: mantém posição fixa quando não há input do piloto
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: POSCTL (Controle de Posição)", throttle_duration_sec=2)
            pass
        
        # Controle de posição em velocidade reduzida (para voos próximos a objetos)
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_POSITION_SLOW:
            # Modo de posição lenta: permite voo mais preciso e controlado
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: POSITION_SLOW (Posição Lenta)", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADOS AUTOMÁTICOS ====================
        # Decolagem automática
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            # Drone está em processo de decolagem automática
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: AUTO_TAKEOFF (Decolagem Automática)", throttle_duration_sec=2)
            pass
        
        # Missão automática
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
            # Drone está executando uma missão automática pré-programada
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: AUTO_MISSION (Missão Automática)", throttle_duration_sec=2)
            pass
        
        # Retorno automático ao ponto de partida (RTL - Return To Launch)
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            # Drone está retornando automaticamente para o ponto de partida
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: AUTO_RTL (Retorno ao Ponto de Partida)", throttle_duration_sec=2)
            pass
        
        # Pouso automático
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # Drone está pousando automaticamente
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: AUTO_LAND (Pouso Automático)", throttle_duration_sec=2)
            pass
        
        # Loiter automático (voar em círculos mantendo posição)
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            # Drone está em modo loiter: voa em círculos mantendo posição fixa
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: AUTO_LOITER (Loiter Automático)", throttle_duration_sec=2)
            pass
        
        # Seguir alvo automaticamente
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
            # Drone está seguindo um alvo automaticamente
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: AUTO_FOLLOW_TARGET (Seguir Alvo)", throttle_duration_sec=2)
            pass
        
        # Órbita automática ao redor de um ponto
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_ORBIT:
            # Drone está orbitando ao redor de um ponto específico
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: ORBIT (Órbita)", throttle_duration_sec=2)
            pass
        
        # Pouso preciso automático (usando sensores de precisão)
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND:
            # Drone está em modo de pouso preciso usando sensores de precisão
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: AUTO_PRECLAND (Pouso Preciso)", throttle_duration_sec=2)
            pass
        
        # Decolagem automática VTOL (para veículos VTOL)
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
            # Veículo VTOL está em processo de decolagem automática
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: AUTO_VTOL_TAKEOFF (Decolagem VTOL)", throttle_duration_sec=2)
            pass
        
        # Descida controlada
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_DESCEND:
            # Drone está em descida controlada
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: DESCEND (Descida)", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADOS EXTERNOS (CUSTOMIZADOS) ====================
        # Estados externos permitem controle customizado por aplicações externas
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL1:
            # Estado externo 1: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: EXTERNAL1", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL2:
            # Estado externo 2: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: EXTERNAL2", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL3:
            # Estado externo 3: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: EXTERNAL3", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL4:
            # Estado externo 4: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: EXTERNAL4", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL5:
            # Estado externo 5: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: EXTERNAL5", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL6:
            # Estado externo 6: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: EXTERNAL6", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL7:
            # Estado externo 7: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: EXTERNAL7", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL8:
            # Estado externo 8: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: EXTERNAL8", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADOS LIVRES (RESERVADOS) ====================
        # Estados livres reservados para uso futuro ou customização
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_FREE1:
            # Estado livre 1: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: FREE1", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_FREE2:
            # Estado livre 2: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: FREE2", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_FREE3:
            # Estado livre 3: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: FREE3", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_FREE4:
            # Estado livre 4: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: FREE4", throttle_duration_sec=2)
            pass
        
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_FREE5:
            # Estado livre 5: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Modos de Navegação: FREE5", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADO DE TERMINAÇÃO ====================
        # Estado de emergência: sistema está sendo terminado
        if self.drone_state.px4.nav_state == VehicleStatus.NAVIGATION_STATE_TERMINATION:
            # Estado de terminação: sistema está sendo desligado por segurança
            self.get_logger().warn(f"{self.PX4_RX_PREFIX}Modos de Navegação: TERMINATION (Terminação de Emergência)", throttle_duration_sec=2)
            pass
        
        # ==================== VERIFICAÇÃO DE ESTADO DESCONHECIDO ====================
        # Verifica se o estado está dentro do range válido
        if self.PX4_nav_state >= VehicleStatus.NAVIGATION_STATE_MAX:
            # Estado fora do range válido: possível erro ou versão incompatível do PX4
            self.get_logger().error(f"{self.PX4_RX_PREFIX}Modos de Navegação inválido ou desconhecido: {self.PX4_nav_state}", throttle_duration_sec=2)
            pass

    def vehicle_local_position_callback(self, msg):
        """
        Processa a mensagem de posição local do drone.
        A posição local é relativa ao ponto de partida (home) e é dada em metros.
        Coordenadas: X (norte), Y (leste), Z (para baixo, negativo = para cima)
        """
        self.drone_state.px4.local_position = msg
        

    def vehicle_global_position_callback(self, msg):
        """
        Processa a mensagem de posição global (GPS) do drone.
        Contém latitude, longitude e altitude em coordenadas globais.
        """
        self.drone_state.px4.global_position = msg
        

    def vehicle_attitude_callback(self, msg):
        """
        Processa a mensagem de atitude (orientação) do drone.
        A atitude é representada como um quaternião (q[0]=w, q[1]=x, q[2]=y, q[3]=z).
        """
        self.drone_state.px4.vehicle_attitude = msg
        
        # Calcula e publica o yaw atual em graus
        q_w = msg.q[0]
        q_x = msg.q[1]
        q_y = msg.q[2]
        q_z = msg.q[3]
        
        # Conversão de quaternião para ângulo de Euler (yaw)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        
        self.drone_state.px4.current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.drone_state.px4.current_yaw_deg = math.degrees(self.drone_state.px4.current_yaw_rad)
        # Normaliza para -180 a 180 graus
        yaw_normalized = self.drone_state.px4.current_yaw_deg % 360
        if yaw_normalized > 180:
            yaw_normalized -= 360
        elif yaw_normalized < -180:
            yaw_normalized += 360
        self.drone_state.px4.current_yaw_deg_normalized = yaw_normalized
        

    def land_detected_callback(self, msg):
        """
        Processa a mensagem de detecção de pouso.
        Indica se o drone detectou que pousou (através de sensores de pressão, acelerômetros, etc.)
        """
        self.drone_state.px4.is_landed = msg.landed


    def _get_command_description(self, command_id: int) -> str:
        """
        Retorna a descrição legível de um comando PX4 baseado no seu ID.
        
        Args:
            command_id: ID numérico do comando
            
        Returns:
            str: Descrição do comando ou "Comando_{ID}" se não encontrado
        """
        # Mapeamento dos comandos mais comuns do PX4
        command_descriptions = {
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: "ARMAR/DESARMAR",
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE: "DEFINIR_MODO",
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF: "DECOLAGEM",
            VehicleCommand.VEHICLE_CMD_NAV_LAND: "POUSO",
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: "RETORNAR_AO_LANÇAMENTO",
            VehicleCommand.VEHICLE_CMD_DO_SET_HOME: "DEFINIR_HOME",
            VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER: "DEFINIR_PARÂMETRO",
            VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION: "TERMINAÇÃO_DE_VOO",
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED: "ALTERAR_VELOCIDADE",
            VehicleCommand.VEHICLE_CMD_DO_SET_ROI: "DEFINIR_ROI",
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL: "CONTROLE_DE_MOUNT",
            # VehicleCommand.VEHICLE_CMD_DO_SET_CAMERA_TRIGGER: "DISPARAR_CÂMERA",
            VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION: "TRANSIÇÃO_VTOL",
            VehicleCommand.VEHICLE_CMD_NAV_VTOL_TAKEOFF: "DECOLAGEM_VTOL",
            VehicleCommand.VEHICLE_CMD_NAV_VTOL_LAND: "POUSO_VTOL",
            # VehicleCommand.VEHICLE_CMD_NAV_GUIDED_ENABLE: "HABILITAR_GUIDADO",
            # VehicleCommand.VEHICLE_CMD_NAV_ORBIT: "ÓRBITA",
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_CALIBRATION: "CALIBRAÇÃO_PREFLIGHT",
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN: "REINICIAR_DESLIGAR",
            VehicleCommand.VEHICLE_CMD_DO_REPOSITION: "REPOSICIONAR",
            VehicleCommand.VEHICLE_CMD_DO_PAUSE_CONTINUE: "PAUSAR_CONTINUAR",
            VehicleCommand.VEHICLE_CMD_DO_ORBIT: "ÓRBITA",
            VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST: "TESTE_DE_MOTOR",
        }
        return command_descriptions.get(command_id, f"Comando_{command_id}")

    def command_ack_callback(self, msg):
        """
        Processa confirmações de comandos enviados ao PX4.
        Apenas loga ACKs para comandos enviados pelo drone_node (rastreados em pending_commands).
        """
        self.drone_state.px4.last_command_ack = msg
        
        # Verifica se o comando foi enviado pelo drone_node
        if msg.command not in self.pending_commands:
            return  # Ignora ACKs de comandos não enviados por nós
        
        # Remove o comando do conjunto de pendentes
        self.pending_commands.discard(msg.command)
        
        result_text = "ACEITO" if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED else "REJEITADO"
        command_description = self._get_command_description(msg.command)
        self.get_logger().info(f"{self.PX4_RX_PREFIX}ACK Comando {command_description} ({msg.command}): {result_text} (código: {msg.result})")

    def home_position_callback(self, msg):
        """
        Processa posição de home (HomePosition) do PX4.
        Atualiza as variáveis de referência HOME em DroneState para uso no RTL e conversões GPS.
        
        Estrutura da mensagem HomePosition:
        - lat, lon, alt: Posição global (GPS)
        - x, y, z: Posição local (NED)
        - yaw: Orientação em radianos
        - valid_alt, valid_hpos, valid_lpos: Flags de validade
        """
        self.drone_state.px4.home_position = msg  # Armazena a mensagem original
        
        # Extrai coordenadas globais (GPS)
        self.drone_state.px4.home_global_lat = msg.lat
        self.drone_state.px4.home_global_lon = msg.lon
        self.drone_state.px4.home_global_alt = msg.alt
        
        # Extrai posição local [x, y, z]
        self.drone_state.px4.home_local_position = [msg.x, msg.y, msg.z]
        
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
        
        self.drone_state.px4.home_yaw_rad = yaw_rad
        self.drone_state.px4.home_yaw_deg = yaw_deg_0_360
        self.drone_state.px4.home_yaw_deg_normalized = yaw_deg_normalized
        
        self.get_logger().info(
            f"{self.PX4_RX_PREFIX}HOME atualizado via PX4: "
            f"GPS=[{self.drone_state.px4.home_global_lat:.6f}, {self.drone_state.px4.home_global_lon:.6f}, {self.drone_state.px4.home_global_alt:.2f}m], "
            f"Local=[{self.drone_state.px4.home_local_position[0]:.2f}, {self.drone_state.px4.home_local_position[1]:.2f}, {self.drone_state.px4.home_local_position[2]:.2f}], "
            f"Yaw={self.drone_state.px4.home_yaw_deg:.1f}°"
        )


    def battery_status_callback(self, msg):
        """
        Processa status da bateria (BatteryStatus)."""

        self.drone_state.px4.battery_status = msg
        self.battery_status_pub.publish(msg)


    def failsafe_flags_callback(self, msg):
        """
        Processa flags de failsafe (FailsafeFlags)."""
        pass


    # ==================================================================
    # SEÇÃO 4: AÇÕES - Funções Principais (comandos para o PX4)
    # ==================================================================

    def publish_vehicle_command(self, command, **params):
        """
        Função central para construir e publicar uma mensagem VehicleCommand para o PX4.
        
        Args:
            command: ID do comando (constantes VehicleCommand.VEHICLE_CMD_*)
            **params: Parâmetros do comando (param1-param7)
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1      # ID do sistema alvo (1 = autopilot)
        msg.target_component = 1   # ID do componente alvo (1 = autopilot)
        msg.source_system = 255    # ID do sistema fonte (255 = companion computer)
        msg.source_component = 1   # ID do componente fonte
        msg.from_external = True   # Indica que o comando vem de fonte externa
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # Timestamp em microssegundos
        
        self.vehicle_command_pub.publish(msg)
        
        # Adiciona ao conjunto de comandos pendentes para rastrear ACK
        self.pending_commands.add(command)
        
        self.get_logger().debug(
            f"{self.PX4_TX_PREFIX}[{self.PX4_TOPIC_VEHICLE_COMMAND}] "
            f"Comando (ID: {command}) publicado com parâmetros: {params}"
        )

    def arm(self):
        """
        Envia o comando para armar os motores do drone.
        Comando: VEHICLE_CMD_COMPONENT_ARM_DISARM com param1=1.0
        """
        self.get_logger().info(
            f"{self.PX4_TX_PREFIX}[{self.PX4_TOPIC_VEHICLE_COMMAND}] "
            "Enviando comando para ARMAR motores..."
        )
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        """
        Envia o comando para desarmar os motores do drone.
        Comando: VEHICLE_CMD_COMPONENT_ARM_DISARM com param1=0.0
        """
        self.get_logger().info(
            f"{self.PX4_TX_PREFIX}[{self.PX4_TOPIC_VEHICLE_COMMAND}] "
            "Enviando comando para DESARMAR motores..."
        )
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def takeoff(self, altitude=None):
        """
        Inicia decolagem até a altitude especificada.
        O drone deve estar armado e no chão.
        
        Args:
            altitude: Altitude alvo em metros. Se None, usa takeoff_altitude padrão.
        """
        if altitude is None:
            altitude = self.drone_state.takeoff_altitude
        
        # Configura trajetória de decolagem (vertical)
        # Armazena posição atual como origem
        current_x = self.drone_state.px4.local_position.x
        current_y = self.drone_state.px4.local_position.y
        current_z = self.drone_state.px4.local_position.z
        
        self.drone_state.origin_local_position = [current_x, current_y, current_z]
        
        # Calcula altitude alvo relativa à posição HOME (não à posição atual)
        # A altitude alvo é em relação ao HOME, onde Z=0 no frame NED
        # Se home_local_position existe, usa como referência; senão usa 0
        home_z = self.drone_state.px4.home_local_position[2] if self.drone_state.px4.home_local_position else 0.0
        target_z = home_z - altitude  # Z negativo = para cima no frame NED
        
        self.drone_state.target_local_position = [current_x, current_y, target_z]
        
        # Mantém yaw do home durante decolagem (normalized = -180/180, deg = 0-360)
        self.drone_state.target_final_yaw_deg = self.drone_state.px4.home_yaw_deg
        self.drone_state.target_final_yaw_deg_normalized = self.drone_state.px4.home_yaw_deg_normalized
        self.drone_state.target_final_yaw_rad = self.drone_state.px4.home_yaw_rad
        self.drone_state.target_direction_yaw_deg = self.drone_state.px4.home_yaw_deg
        self.drone_state.target_direction_yaw_deg_normalized = self.drone_state.px4.home_yaw_deg_normalized
        self.drone_state.target_direction_yaw_rad = self.drone_state.px4.home_yaw_rad
        
        # Define flag de comando (será processado por verifica_mudanca_de_estado_drone())
        self.drone_state.command_takeoff_requested = True
        
        current_alt = -current_z  # Converte para altitude positiva
        self.get_logger().info(
            f"TAKEOFF solicitado: altitude atual={current_alt:.2f}m, "
            f"altitude alvo={altitude}m (relativa ao HOME)"
        )



    def goto(self, lat=None, lon=None, alt=None, yaw=None):
        """
        Move o drone para uma posição específica usando coordenadas globais.
        Converte as coordenadas globais para locais e armazena para uso nos cálculos de trajetória.
        O drone deve estar armado e em modo Offboard para que isso funcione.
        
        Sequência de execução (completa):
        1. Rotaciona o drone para apontar na direção do destino
        2. Move o drone até a posição alvo
        3. Rotaciona o drone para o yaw alvo (se especificado)
        
        Casos especiais com NaN:
        - lat/lon NaN, alt válido: pula rotação inicial, apenas muda altitude
        - lat/lon/alt NaN, yaw válido: vai direto para rotação final (apenas gira)
        - Todos NaN: comando ignorado
        
        Args:
            lat: Latitude alvo (graus) - pode ser NaN para manter posição atual
            lon: Longitude alvo (graus) - pode ser NaN para manter posição atual
            alt: Altitude alvo GLOBAL em metros (MSL) - pode ser NaN para manter altitude atual
            yaw: Yaw alvo final (graus, -180 a 180) - opcional, None/NaN mantém o yaw atual
        """
        # Verifica se lat/lon são NaN
        lat_is_nan = lat is None or (isinstance(lat, float) and math.isnan(lat))
        lon_is_nan = lon is None or (isinstance(lon, float) and math.isnan(lon))
        alt_is_nan = alt is None or (isinstance(alt, float) and math.isnan(alt))
        yaw_is_nan = yaw is None or (isinstance(yaw, float) and math.isnan(yaw))
        
        # Se todos são NaN, não há nada a fazer
        if lat_is_nan and lon_is_nan and alt_is_nan and yaw_is_nan:
            self.get_logger().error("Não é possível executar GOTO: Todos os parâmetros são NaN/None.")
            return
        
        # Armazena coordenadas globais de origem
        self.drone_state.origin_latitude = self.drone_state.px4.global_position.lat
        self.drone_state.origin_longitude = self.drone_state.px4.global_position.lon
        self.drone_state.origin_altitude = self.drone_state.px4.global_position.alt
        
        # Armazena posição local de origem
        self.drone_state.origin_local_position = [
            self.drone_state.px4.local_position.x, 
            self.drone_state.px4.local_position.y, 
            self.drone_state.px4.local_position.z
        ]
        
        # Define target_latitude/longitude/altitude (usando NaN para indicar "manter atual")
        self.drone_state.target_latitude = lat if not lat_is_nan else float('nan')
        self.drone_state.target_longitude = lon if not lon_is_nan else float('nan')
        self.drone_state.target_altitude = alt if not alt_is_nan else float('nan')
        
        # Calcula target_local_position baseado nos valores disponíveis
        if lat_is_nan or lon_is_nan:
            # lat/lon NaN: mantém posição X/Y atual
            target_local_x = self.drone_state.px4.local_position.x
            target_local_y = self.drone_state.px4.local_position.y
        else:
            # Converte lat/lon para local
            target_local = self.global_to_local_position(lat, lon, self.drone_state.px4.global_position.alt)
            if target_local is None:
                self.get_logger().error("Não é possível executar GOTO: Erro ao converter coordenadas globais para locais.")
                return
            target_local_x = target_local[0]
            target_local_y = target_local[1]
        
        if alt_is_nan:
            # alt NaN: mantém altitude atual
            target_local_z = self.drone_state.px4.local_position.z
        else:
            # Converte altitude GLOBAL para local (z é negativo em NED)
            # Calcula z baseado na diferença entre altitude global alvo e altitude global do home
            alt_diff = alt - self.drone_state.px4.home_global_alt
            target_local_z = -alt_diff
        
        self.drone_state.target_local_position = [target_local_x, target_local_y, target_local_z]
        
        # Calcula distância inicial para cálculo de progresso
        dx = target_local_x - self.drone_state.px4.local_position.x
        dy = target_local_y - self.drone_state.px4.local_position.y
        dz = target_local_z - self.drone_state.px4.local_position.z
        self.drone_state.initial_distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)
        # Armazena yaw alvo (pode ser None/NaN)
        if not yaw_is_nan:
            # yaw é recebido na faixa -180 a 180 (normalized)
            yaw_normalized = yaw
            yaw_0_360 = yaw if yaw >= 0 else yaw + 360
            self.drone_state.target_final_yaw_deg = yaw_0_360
            self.drone_state.target_final_yaw_deg_normalized = yaw_normalized
            self.drone_state.target_final_yaw_rad = math.radians(yaw_normalized)
        else:
            self.drone_state.target_final_yaw_deg = None
            self.drone_state.target_final_yaw_deg_normalized = None
            self.drone_state.target_final_yaw_rad = None
        
        # Calcula o yaw de direção apenas se houver movimento horizontal
        if not (lat_is_nan or lon_is_nan):
            dx = target_local_x - self.drone_state.px4.local_position.x
            dy = target_local_y - self.drone_state.px4.local_position.y
            target_direction_yaw = math.degrees(math.atan2(dy, dx))
            # Normaliza para -180 a 180
            if target_direction_yaw > 180:
                target_direction_yaw -= 360
            elif target_direction_yaw < -180:
                target_direction_yaw += 360
            # Armazena ambas versões (normalized = -180/180, deg = 0-360)
            self.drone_state.target_direction_yaw_deg_normalized = target_direction_yaw
            target_direction_yaw_0_360 = target_direction_yaw if target_direction_yaw >= 0 else target_direction_yaw + 360
            self.drone_state.target_direction_yaw_deg = target_direction_yaw_0_360
            self.drone_state.target_direction_yaw_rad = math.radians(target_direction_yaw)
        else:
            # Sem movimento horizontal, não precisa de yaw de direção
            self.drone_state.target_direction_yaw_deg = None
            self.drone_state.target_direction_yaw_deg_normalized = None
            self.drone_state.target_direction_yaw_rad = None
        
        # Define flag de comando (será processado por verifica_mudanca_de_estado_drone())
        self.drone_state.command_goto_requested = True
        
        # Log informativo
        lat_str = f"{lat:.6f}" if not lat_is_nan else "NaN"
        lon_str = f"{lon:.6f}" if not lon_is_nan else "NaN"
        alt_str = f"{alt:.2f}" if not alt_is_nan else "NaN"
        yaw_str = f"{yaw:.1f}°" if not yaw_is_nan else "NaN"
        
        self.get_logger().info(
            f"GOTO solicitado: lat={lat_str}, lon={lon_str}, alt={alt_str}, yaw={yaw_str}"
        )
        self.get_logger().info(
            f"GOTO: target_local=[{target_local_x:.2f}, {target_local_y:.2f}, {target_local_z:.2f}], "
            f"direction_yaw={self.drone_state.target_direction_yaw_deg if self.drone_state.target_direction_yaw_deg is not None else 'N/A'}"
        )


    def goto_focus(self, lat=None, lon=None, alt=None, focus_lat=None, focus_lon=None):
        """
        Move o drone para uma posição específica mantendo o yaw apontando para um ponto de foco.
        Durante toda a trajetória, o drone mantém o yaw direcionado para o ponto de foco.
        
        Sequência de execução:
        1. Rotaciona para apontar ao foco (VOANDO_GIRANDO_COM_FOCO)
        2. Move até o destino mantendo yaw no foco (VOANDO_A_CAMINHO_COM_FOCO)
        3. Aguarda estabilização no destino
        
        Args:
            lat: Latitude alvo (graus) - obrigatório
            lon: Longitude alvo (graus) - obrigatório
            alt: Altitude alvo (metros) - obrigatório
            focus_lat: Latitude do ponto de foco (graus) - obrigatório
            focus_lon: Longitude do ponto de foco (graus) - obrigatório
        """
        if lat is None or lon is None or alt is None:
            self.get_logger().error("Não é possível executar GOTO_FOCUS: Coordenadas de destino incompletas (lat, lon, alt são obrigatórias).")
            return
        
        if focus_lat is None or focus_lon is None:
            self.get_logger().error("Não é possível executar GOTO_FOCUS: Coordenadas de foco incompletas (focus_lat, focus_lon são obrigatórias).")
            return
        
        # Armazena coordenadas globais para referência
        self.drone_state.origin_latitude = self.drone_state.px4.global_position.lat
        self.drone_state.origin_longitude = self.drone_state.px4.global_position.lon
        self.drone_state.origin_altitude = self.drone_state.px4.global_position.alt
        self.drone_state.target_latitude = lat
        self.drone_state.target_longitude = lon
        self.drone_state.target_altitude = alt
        self.drone_state.focus_latitude = focus_lat
        self.drone_state.focus_longitude = focus_lon
        
        # Converte coordenadas globais para locais
        self.drone_state.origin_local_position = [
            self.drone_state.px4.local_position.x, 
            self.drone_state.px4.local_position.y, 
            self.drone_state.px4.local_position.z
        ]
        
        target_local = self.global_to_local_position(lat, lon, alt)
        if target_local is None:
            self.get_logger().error("Não é possível executar GOTO_FOCUS: Erro ao converter coordenadas de destino para locais.")
            return
        
        self.drone_state.target_local_position = target_local
        
        # Converte ponto de foco para local (usando altitude atual para simplificar)
        focus_local = self.global_to_local_position(focus_lat, focus_lon, -self.drone_state.px4.local_position.z)
        if focus_local is None:
            self.get_logger().error("Não é possível executar GOTO_FOCUS: Erro ao converter coordenadas de foco para locais.")
            return
        
        self.drone_state.focus_local_position = focus_local
        
        # Define flag de comando (será processado por verifica_mudanca_de_estado_drone())
        self.drone_state.command_goto_focus_requested = True
        
        self.get_logger().info(
            f"GOTO_FOCUS solicitado: destino=[{lat:.6f}, {lon:.6f}, {alt:.2f}], "
            f"foco=[{focus_lat:.6f}, {focus_lon:.6f}]"
        )
        self.get_logger().info(
            f"GOTO_FOCUS: destino local=[{target_local[0]:.2f}, {target_local[1]:.2f}, {target_local[2]:.2f}], "
            f"foco local=[{focus_local[0]:.2f}, {focus_local[1]:.2f}]"
        )


    def land(self):
        """
        Inicia pouso usando o comando nativo do PX4 (VEHICLE_CMD_NAV_LAND).
        O PX4 muda temporariamente para modo AUTO_LAND e executa o pouso de forma autônoma.
        O drone desce na posição atual com taxa de descida controlada pelo PX4.
        Após pousar, se setpoints Offboard estiverem sendo publicados, retorna ao modo Offboard.
        """
        self.get_logger().info(
            f"{self.PX4_TX_PREFIX}[{self.PX4_TOPIC_VEHICLE_COMMAND}] "
            "Enviando comando LAND nativo do PX4 (pouso na posição atual)..."
        )
        
        # Envia comando de pouso nativo - o PX4 cuida de tudo
        # O drone pousa na posição atual, não vai para home
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
        # Define flag para a máquina de estados saber que está pousando
        self.drone_state.command_land_requested = True
        
        self.get_logger().info(
            f"LAND nativo solicitado: posição atual X={self.drone_state.px4.local_position.x:.2f}m, "
            f"Y={self.drone_state.px4.local_position.y:.2f}m, "
            f"Alt={-self.drone_state.px4.local_position.z:.2f}m"
        )



    def rtl(self):
        """
        Inicia retorno à base via Offboard com sequência:
        1. RETORNANDO_GIRANDO_INICIO: Gira para apontar na direção do home
        2. RETORNANDO_A_CAMINHO: Sobe até a altitude RTL enquanto vai para o home (X=0, Y=0)
        3. RETORNANDO_GIRANDO_FIM: Gira para o yaw final do home (home_yaw)
        4. POUSANDO: Pousa usando comando nativo do PX4
        
        Nota: O PX4 desarma automaticamente após pousar (parâmetro COM_DISARM_LAND).
        """
        self.get_logger().info(
            f"Iniciando RTL via Offboard: girar → subir {self.drone_state.rtl_altitude}m → ir para home → girar para yaw home → pousar..."
        )
        
        # Configura origem da trajetória
        current_x = self.drone_state.px4.local_position.x
        current_y = self.drone_state.px4.local_position.y
        current_z = self.drone_state.px4.local_position.z
        
        self.drone_state.origin_local_position = [current_x, current_y, current_z]
        
        # Alvo: posição HOME na altitude RTL
        # Usa home_local_position se disponível, senão assume (0, 0)
        if self.drone_state.px4.home_local_position is not None:
            home_x = self.drone_state.px4.home_local_position[0]
            home_y = self.drone_state.px4.home_local_position[1]
            home_z = self.drone_state.px4.home_local_position[2]
        else:
            home_x = 0.0
            home_y = 0.0
            home_z = 0.0
            self.get_logger().warn("RTL: home_local_position não disponível, usando (0, 0, 0)")
        
        # Altitude RTL relativa ao HOME (Z negativo = para cima no frame NED)
        target_z = home_z - self.drone_state.rtl_altitude
        
        self.drone_state.target_local_position = [home_x, home_y, target_z]
        
        # Calcula yaw de direção para apontar para home
        dx = home_x - current_x
        dy = home_y - current_y
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            target_direction_yaw = math.degrees(math.atan2(dy, dx))
            # Normaliza para -180 a 180
            if target_direction_yaw > 180:
                target_direction_yaw -= 360
            elif target_direction_yaw < -180:
                target_direction_yaw += 360
            # Armazena ambas versões (normalized = -180/180, deg = 0-360)
            self.drone_state.target_direction_yaw_deg_normalized = target_direction_yaw
            target_direction_yaw_0_360 = target_direction_yaw if target_direction_yaw >= 0 else target_direction_yaw + 360
            self.drone_state.target_direction_yaw_deg = target_direction_yaw_0_360
            self.drone_state.target_direction_yaw_rad = math.radians(target_direction_yaw)
        else:
            # Já está no home, mantém yaw atual
            self.drone_state.target_direction_yaw_deg_normalized = self.drone_state.px4.current_yaw_deg_normalized
            current_yaw_0_360 = self.drone_state.px4.current_yaw_deg_normalized
            if current_yaw_0_360 < 0:
                current_yaw_0_360 += 360
            self.drone_state.target_direction_yaw_deg = current_yaw_0_360
            self.drone_state.target_direction_yaw_rad = self.drone_state.px4.current_yaw_rad
        
        # Yaw final será o yaw do home (normalized = -180/180, deg = 0-360)
        self.drone_state.target_final_yaw_deg = self.drone_state.px4.home_yaw_deg
        self.drone_state.target_final_yaw_deg_normalized = self.drone_state.px4.home_yaw_deg_normalized
        self.drone_state.target_final_yaw_rad = self.drone_state.px4.home_yaw_rad
        
        # Define flag de comando (será processado por verifica_mudanca_de_estado_drone())
        self.drone_state.command_rtl_requested = True
        
        self.get_logger().info(
            f"RTL solicitado: origem=[{current_x:.2f}, {current_y:.2f}, {-current_z:.2f}m], "
            f"alvo=home [{home_x:.2f}, {home_y:.2f}, {self.drone_state.rtl_altitude}m], "
            f"yaw_direção={self.drone_state.target_direction_yaw_deg:.1f}°, home_yaw={self.drone_state.px4.home_yaw_deg:.1f}°"
        )


    def stop(self):
        """
        Para o drone no ar durante uma missão de GOTO, GOTO_FOCUS ou RTL.
        
        Ação:
        1. Reseta todas as variáveis de trajetória (via reseta_variaveis_estado)
        2. Armazena a posição atual como última posição estática para hover estável
        3. Muda o estado para VOANDO_PRONTO para aguardar novo comando
        
        Este comando só pode ser recebido durante estados de movimento:
        - GOTO: VOANDO_GIRANDO_INICIO, VOANDO_A_CAMINHO, VOANDO_GIRANDO_FIM
        - GOTO_FOCUS: VOANDO_GIRANDO_COM_FOCO, VOANDO_A_CAMINHO_COM_FOCO
        - RTL: RETORNANDO_GIRANDO_INICIO, RETORNANDO_A_CAMINHO, RETORNANDO_GIRANDO_FIM
        """
        current_state = self.drone_state.state.name
        self.get_logger().info(f"STOP recebido durante estado {current_state} - parando drone...")
        
        # Reseta todas as variáveis de trajetória e armazena posição para hover
        self.drone_state.reseta_variaveis_estado()
        
        # Muda para estado VOANDO_PRONTO para aguardar novo comando
        self.drone_state.mudar_estado(DroneStateDescription.VOANDO_PRONTO)
        
        self.get_logger().info(
            f"STOP executado: drone parado em hover, aguardando novo comando. "
            f"Posição estática: [{self.drone_state.last_static_position[0]:.2f}, "
            f"{self.drone_state.last_static_position[1]:.2f}, {-self.drone_state.last_static_position[2]:.2f}m]"
        )


    # ==================================================================
    # SEÇÃO 5: VERIFICAÇÃO - Funções Auxiliares
    # ==================================================================

    def is_at_target_position(self):
        """
        Verifica se o drone está próximo da posição alvo usando coordenadas locais.
        Também verifica se o yaw está no alvo (se especificado).
        
        Returns:
            bool: True se o drone está dentro da tolerância do alvo (posição e yaw), False caso contrário
        """
        # Calcula distância usando coordenadas locais diretamente
        dx = self.drone_state.px4.local_position.x - self.drone_state.target_local_position[0]
        dy = self.drone_state.px4.local_position.y - self.drone_state.target_local_position[1]
        dz = self.drone_state.px4.local_position.z - self.drone_state.target_local_position[2]
        
        distance_2d = math.sqrt(dx**2 + dy**2)
        distance_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Verifica se está na posição alvo
        position_reached = distance_3d <= self.drone_state.position_tolerance
        
        # Verifica se o yaw está no alvo (se especificado)
        yaw_reached = True
        yaw_diff = None
        if self.drone_state.target_final_yaw_deg is not None:
            yaw_diff = self.drone_state.px4.current_yaw_deg_normalized - self.drone_state.target_final_yaw_deg
            # Normaliza para lidar com transição 360°->0°
            if yaw_diff < -180:
                yaw_diff += 360
            if yaw_diff > 180:
                yaw_diff -= 360
            yaw_reached = abs(yaw_diff) <= self.drone_state.yaw_tolerance_deg
        
        self.get_logger().info(
            f"Verificação de chegada: distância={distance_3d:.2f}m (tol: {self.drone_state.position_tolerance}m), "
            f"yaw_diff={abs(yaw_diff) if yaw_diff is not None else 'N/A'}° (tol: {self.drone_state.yaw_tolerance_deg}°)",
            throttle_duration_sec=2.0
        )
        
        return position_reached and yaw_reached

    # ==================================================================
    # SEÇÃO 6: CONTROLE DE MODO - Funções Auxiliares
    # ==================================================================

    def set_offboard_mode(self):
        """
        Envia o comando para mudar o modo de voo para Offboard.
        Isso só funcionará se setpoints Offboard estiverem sendo publicados continuamente.
        """
        self.get_logger().info(
            f"{self.PX4_TX_PREFIX}[{self.PX4_TOPIC_VEHICLE_COMMAND}] "
            "Enviando comando para mudar para o modo OFFBOARD..."
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0,  # Custom mode enabled
            param2=6.0   # Offboard mode
        )

    def set_position_mode(self):
        """
        Envia o comando para mudar o modo de voo para Position (POSCTL).
        O drone irá parar no ar mantendo a posição atual.
        """
        self.get_logger().info(
            f"{self.PX4_TX_PREFIX}[{self.PX4_TOPIC_VEHICLE_COMMAND}] "
            "Enviando comando para mudar para o modo POSITION..."
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # Custom mode enabled
            param2=3.0   # Position mode (POSCTL)
        )




# ==================================================================================================
# ENUM UNIFICADO DE ESTADOS DO DRONE
# ==================================================================================================

class DroneStateDescription(IntEnum):
    """
    Enum unificado que representa todos os estados possíveis do drone.
    Substitui os antigos DroneStateEnum + TrajectorySubState.
    Usa IntEnum para comparações mais rápidas. Use .name para obter o nome como string.
    """
    # Estados em solo (0-9)
    POUSADO_DESARMADO = 0     # No chão, desarmado
    POUSADO_ARMADO = 1        # No chão, armado
    OFFBOARD_DESATIVADO = 2   # Fora do modo offboard (modo manual, POSCTL, etc.)
    
    # Estados de voo estável (10-19)
    VOANDO_PRONTO = 10        # Voando estável, aguardando comando (hover)
    
    # Estados de trajetória/movimento GOTO (20-29)
    VOANDO_DECOLANDO = 20     # Subindo até altitude de decolagem
    VOANDO_GIRANDO_INICIO = 21  # Girando para direção do target
    VOANDO_A_CAMINHO = 22     # Voando em direção ao target
    VOANDO_GIRANDO_FIM = 23   # Girando para yaw final
    VOANDO_GIRANDO_COM_FOCO = 25   # Girando para apontar para o focus antes de mover
    VOANDO_A_CAMINHO_COM_FOCO = 26  # Voando para target apontando para ponto de focus
    
    # Estados de retorno ao home RTL (30-39)
    RETORNANDO_GIRANDO_INICIO = 30  # Girando para apontar para HOME
    RETORNANDO_A_CAMINHO = 31       # Voando em direção ao HOME
    RETORNANDO_GIRANDO_FIM = 32     # Girando para yaw final do HOME
    
    # Estado de pouso (40)
    POUSANDO = 40             # Descendo para pousar (usado por RTL e LAND)
    
    # Emergência (99)
    EMERGENCIA = 99           # Failsafe/emergência ativa


# ==================================================================================================
# CLASSE DE ESTADO DO PX4
# ==================================================================================================

class DroneStatePX4:
    """
    Encapsula o estado interno do PX4 (autopilot).
    Contém todas as variáveis relacionadas à telemetria e status do PX4.
    """
    
    def __init__(self):
        """Inicializa todas as variáveis de estado do PX4 com valores padrão."""
        # Status e controle do veículo
        self.nav_state = None            # Modos de Navegação atual do PX4
        self.local_position = None        # Última posição local (X, Y, Z) em metros
        self.home_position = None         # Posição de home (ponto de partida)
        self.global_position = None       # Última posição global (latitude, longitude, altitude)
        self.vehicle_attitude = None      # Última atitude (orientação) do drone
        self.current_yaw_deg = 0.0        # Yaw atual do drone (em graus)
        self.current_yaw_rad = 0.0        # Yaw atual do drone (em radianos)
        self.current_yaw_deg_normalized = 0.0 # Yaw atual do drone (em graus) normalizado (-180 a 180)
        self.current_control_mode = None   # Modo de controle atual do PX4
        self.is_armed = False             # Flag booleano para o estado de armamento
        self.is_landed = False            # Flag booleano para o estado de pouso
        self.last_command_ack = None      # Última confirmação de comando recebida do PX4
        self.battery_status = None        # Status da bateria
        
        # --- Referência HOME (atualizada via home_position_callback) ---
        # Estas variáveis são atualizadas quando o PX4 envia HomePosition
        self.home_global_lat = None       # Latitude do HOME (onde local X=0)
        self.home_global_lon = None       # Longitude do HOME (onde local Y=0)
        self.home_global_alt = None       # Altitude do HOME (onde local Z=0)
        self.home_local_position = None   # Posição local do HOME [x, y, z] (deve ser ~[0,0,0])
        self.home_yaw_deg = None          # Yaw do drone ao armar (graus, 0 a 360)
        self.home_yaw_deg_normalized = None  # Yaw do drone ao armar (graus, -180 a 180)
        self.home_yaw_rad = None          # Yaw do drone ao armar (radianos, -pi a pi)


# ==================================================================================================
# MÁQUINA DE ESTADOS DO DRONE
# ==================================================================================================

class DroneState:
    """
    Máquina de estados do drone com enum unificado.
    Gerencia transições automáticas baseadas em condições e valida comandos por estado.
    
    RESPONSABILIDADES:
    - Monitorar condições do drone e transicionar estados automaticamente
    - Validar se comandos são permitidos no estado atual
    - Fornecer informações de estado para publicação de status
    - Manter histórico de transições para debug
    """
    
    # Mapeamento de comandos para estados permitidos
    VALID_COMMANDS = {
        "ARM": [DroneStateDescription.POUSADO_DESARMADO],
        "TAKEOFF": [DroneStateDescription.POUSADO_ARMADO],
        "GOTO": [DroneStateDescription.VOANDO_PRONTO],
        "GOTO_FOCUS": [DroneStateDescription.VOANDO_PRONTO],
        "LAND": [DroneStateDescription.VOANDO_PRONTO],
        "RTL": [DroneStateDescription.VOANDO_PRONTO],
        # STOP pode ser recebido durante qualquer fase de GOTO, GOTO_FOCUS ou RTL
        "STOP": [
            DroneStateDescription.VOANDO_GIRANDO_INICIO,
            DroneStateDescription.VOANDO_A_CAMINHO,
            DroneStateDescription.VOANDO_GIRANDO_FIM,
            DroneStateDescription.VOANDO_GIRANDO_COM_FOCO,
            DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO,
            DroneStateDescription.RETORNANDO_GIRANDO_INICIO,
            DroneStateDescription.RETORNANDO_A_CAMINHO,
            DroneStateDescription.RETORNANDO_GIRANDO_FIM,
        ],
    }
    
    def __init__(self, node: 'DroneNode'):
        """
        Inicializa a máquina de estados.
        
        Args:
            node: Referência ao DroneNode para acessar métodos do nó (get_logger, reseta_variaveis_estado, etc.)
        """
        self.node = node
        
        # Cria instância do estado PX4
        self.px4 = DroneStatePX4()
        
        # Estado da máquina de estados
        self.state = DroneStateDescription.OFFBOARD_DESATIVADO
        self.state_entry_time = time.time()
        
        # --- Armazenamento de Estado Interno do Drone ---
        # Indica se o drone está publicando setpoints de trajetória de movimento (moving),
        # ao invés de manter posição (static).
        self.on_trajectory = False
        self.trajectory_start_time = None     # Timestamp do início do movimento (em segundos)

        # --- Parâmetros de Trajetória Simplificada ---
        self.step_distance = 5.0              # Distância do próximo passo (metros)
        self.yaw_step_deg = 15.0              # Passo máximo de rotação em graus (para giros incrementais)

        # --- Parâmetros de Navegação ---
        self.origin_local_position = None     # Posição de origem quando inicia trajetória [x, y, z]
        self.target_local_position = None     # Posição alvo para trajetória [x, y, z]
        self.target_latitude = None           # Latitude alvo para trajetória (graus)
        self.target_longitude = None          # Longitude alvo para trajetória (graus)
        self.target_altitude = None           # Altitude alvo para trajetória (metros)
        self.origin_latitude = None           # Latitude de origem quando inicia trajetória (graus)
        self.origin_longitude = None          # Longitude de origem quando inicia trajetória (graus)
        self.origin_altitude = None           # Altitude de origem quando inicia trajetória (metros)
        self.target_final_yaw_deg = None            # Yaw alvo final (graus, 0 a 360) - None significa manter yaw atual
        self.target_final_yaw_deg_normalized = None # Yaw alvo final normalizado (graus, -180 a 180)
        self.target_final_yaw_rad = None            # Yaw alvo final (radianos, -pi a pi)
        self.position_tolerance = 0.15        # Tolerância em metros para considerar que chegou ao destino
        self.yaw_tolerance_deg = 2.0          # Tolerância em graus para o yaw (apontamento)
        
        # --- Estados da Trajetória ---
        # A fase da trajetória agora é representada diretamente por DroneStateDescription (state machine)
        self.target_direction_yaw_deg = None         # Yaw necessário para apontar na direção do destino (graus, 0 a 360)
        self.target_direction_yaw_deg_normalized = None  # Yaw necessário para apontar na direção do destino (graus, -180 a 180)
        self.target_direction_yaw_rad = None         # Yaw necessário para apontar na direção do destino (radianos)
        self.takeoff_altitude = 2.5           # Altitude padrão de decolagem (metros)
        self.rtl_altitude = 30.0              # Altitude do RTL em metros (sobe antes de ir para home)
        self.yaw_aligned_time = None          # Timestamp de quando o yaw foi alinhado pela primeira vez (para delay de estabilização)
        self.yaw_stabilization_delay = 3.0    # Tempo em segundos para aguardar após alinhar o yaw antes de iniciar movimento
        self.initial_distance_to_target = None  # Distância inicial até o alvo (para cálculo de progresso)
        
        # --- Última Posição Estática para Hover ---
        # Armazena a última posição estática ao finalizar um movimento para evitar instabilidade
        # durante o hover (nunca envia local_position diretamente ao trajectory)
        self.last_static_position = None       # Última posição estática conhecida [x, y, z]
        self.last_static_yaw_deg = None        # Último yaw estático conhecido (graus, 0 a 360)
        self.last_static_yaw_deg_normalized = None  # Último yaw estático conhecido (graus, -180 a 180)
        self.last_static_yaw_rad = None        # Último yaw estático conhecido (radianos)
        
        # --- Flags de Comandos Recebidos da FSM ---
        # Estas flags são definidas pelos métodos de comando e processadas por verifica_mudanca_de_estado_drone()
        # Nota: Os valores dos comandos (lat, lon, alt, yaw) são armazenados em target_* e origin_*
        self.command_takeoff_requested = False
        self.command_goto_requested = False
        self.command_goto_focus_requested = False
        self.focus_latitude = None
        self.focus_longitude = None
        self.focus_local_position = None
        self.focus_yaw_deg = None         # Yaw para apontar ao foco (graus, 0 a 360)
        self.focus_yaw_deg_normalized = None  # Yaw para apontar ao foco (graus, -180 a 180)
        self.focus_yaw_rad = None
        self.command_land_requested = False
        self.command_rtl_requested = False
    
    def reseta_variaveis_estado(self):
        """
        Para a trajetória e limpa todas as variáveis relacionadas.
        Armazena a posição atual em last_static_position para manter estabilidade no hover.
        """
        # Armazena a última posição estática antes de resetar, para usar no hover
        if self.px4.local_position is not None:
            self.last_static_position = [
                self.px4.local_position.x,
                self.px4.local_position.y,
                self.px4.local_position.z
            ]
            # Armazena yaw estático (normalized = -180/180, deg = 0-360)
            self.last_static_yaw_deg_normalized = self.px4.current_yaw_deg_normalized
            self.last_static_yaw_deg = self.px4.current_yaw_deg_normalized
            if self.last_static_yaw_deg < 0:
                self.last_static_yaw_deg += 360
            self.last_static_yaw_rad = self.px4.current_yaw_rad
            self.node.get_logger().info(
                f"RESETANDO VARIAVEIS DE ESTADO: Posição estática armazenada para hover: "
                f"[{self.last_static_position[0]:.2f}, {self.last_static_position[1]:.2f}, {self.last_static_position[2]:.2f}]"
            )
        self.on_trajectory = False
        self.target_latitude = None
        self.target_longitude = None
        self.target_altitude = None
        self.origin_latitude = None
        self.origin_longitude = None
        self.origin_altitude = None
        self.origin_local_position = None
        self.target_local_position = None
        self.target_final_yaw_deg = None
        self.target_final_yaw_deg_normalized = None
        self.target_final_yaw_rad = None
        self.target_direction_yaw_deg = None
        self.target_direction_yaw_deg_normalized = None
        self.target_direction_yaw_rad = None
        self.trajectory_start_time = None
        self.focus_latitude = None
        self.focus_longitude = None
        self.focus_local_position = None
        self.focus_yaw_deg = None
        self.focus_yaw_deg_normalized = None
        self.focus_yaw_rad = None
    
    def verifica_mudanca_de_estado_drone(self):
        """
        Verifica e executa transições automáticas baseadas nas condições do drone.
        Usa o enum unificado DroneStateDescription com match/case.
        
        ESTRUTURA:
        1. Verifica condição de emergência (prioridade absoluta)
        2. Verifica se desarmou (para todos os estados exceto POUSADO_DESARMADO)
        3. Usa match/case no estado atual para processar verificações e transições
        """
        # =================================================
        # CÓPIAS LOCAIS PARA EVITAR RACE CONDITIONS
        # Armazena o estado e variáveis críticas no início para garantir
        # consistência durante toda a execução do step
        # =================================================
        current_state = self.state
        
        # Variáveis do PX4 usadas para decisões críticas
        px4_is_armed = self.px4.is_armed
        px4_is_landed = self.px4.is_landed
        px4_local_position = self.px4.local_position
        px4_current_yaw_deg_normalized = self.px4.current_yaw_deg_normalized
        px4_current_yaw_rad = self.px4.current_yaw_rad
        
        # Flags de comandos recebidos da FSM
        cmd_takeoff_requested = self.command_takeoff_requested
        cmd_goto_requested = self.command_goto_requested
        cmd_goto_focus_requested = self.command_goto_focus_requested
        cmd_land_requested = self.command_land_requested
        cmd_rtl_requested = self.command_rtl_requested
        
        # Variáveis de posição alvo e direção
        local_target_local_position = self.target_local_position
        local_target_direction_yaw_deg = self.target_direction_yaw_deg
        local_target_final_yaw_deg = self.target_final_yaw_deg
        local_focus_local_position = self.focus_local_position
        
        # === VERIFICAÇÃO GLOBAL: Modo Offboard ===
        # Se o PX4:
        #   1. não está em modo offboard
        #   2. não está em modo auto_land
        # E o Drone State:
        #   3. não está no estado OFFBOARD_DESATIVADO
        #   4. não está no estado POUSANDO
        # Vai para OFFBOARD_DESATIVADO
        if self.px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
           self.px4.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND and \
           current_state != DroneStateDescription.OFFBOARD_DESATIVADO and \
           current_state != DroneStateDescription.POUSANDO:
            self.reseta_variaveis_estado()
            self.node.get_logger().error(f"ENTROU OFFBOARD_DESATIVADO: self.px4.nav_state = {self.px4.nav_state} e current_state = {current_state.name}")
            self.mudar_estado(DroneStateDescription.OFFBOARD_DESATIVADO)
        
        # === VERIFICAÇÃO GLOBAL: Emergência ===
        self.verifica_condicao_de_emergencia()
        
        # === VERIFICAÇÃO GLOBAL: Desarmamento ===
        # Se o drone desarmou e não está no estado POUSADO_DESARMADO ou OFFBOARD_DESATIVADO, para tudo e vai para POUSADO_DESARMADO
        if not px4_is_armed and \
           current_state != DroneStateDescription.POUSADO_DESARMADO and \
           current_state != DroneStateDescription.OFFBOARD_DESATIVADO:
            self.reseta_variaveis_estado()
            self.mudar_estado(DroneStateDescription.POUSADO_DESARMADO)

        
        # === MATCH/CASE PRINCIPAL: Processa estado atual ===
        match current_state:
            
            case DroneStateDescription.OFFBOARD_DESATIVADO:
                # Aguarda retorno ao modo OFFBOARD
                if self.px4.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

                    # Se voltou para offboard, verifica se está armado para decidir próximo estado
                    if px4_is_armed:
                        if px4_is_landed:
                            self.mudar_estado(DroneStateDescription.POUSADO_ARMADO)
                        else:
                            self.mudar_estado(DroneStateDescription.VOANDO_PRONTO)
                    else:
                        self.mudar_estado(DroneStateDescription.POUSADO_DESARMADO)
                    return
            
            case DroneStateDescription.POUSADO_DESARMADO:

                # Verifica se Drone armou os motores
                if px4_is_armed:
                    self.mudar_estado(DroneStateDescription.POUSADO_ARMADO)
                    return
            
            case DroneStateDescription.POUSADO_ARMADO:
                
                # Verificar flag de takeoff
                if cmd_takeoff_requested:
                    self.on_trajectory = True
                    self.trajectory_start_time = time.time()
                    self.command_takeoff_requested = False
                    self.mudar_estado(DroneStateDescription.VOANDO_DECOLANDO)
                    return
                
                # Verificar se drone não está pousado: passa para VOANDO_PRONTO
                if not px4_is_landed:
                    self.node.get_logger().info(
                        "Drone em POUSADO_ARMADO mas não está pousado. "
                        "Transicionando para VOANDO_PRONTO."
                    )
                    self.reseta_variaveis_estado()
                    self.mudar_estado(DroneStateDescription.VOANDO_PRONTO)
                    return
                
            case DroneStateDescription.VOANDO_PRONTO:
                
                # Verificar flag de goto - SEMPRE vai para GIRANDO_INICIO
                if cmd_goto_requested:
                    # Armazena posição estática atual para referência
                    if px4_local_position is not None:
                        self.last_static_position = [
                            px4_local_position.x,
                            px4_local_position.y,
                            px4_local_position.z
                        ]
                        self.last_static_yaw_deg_normalized = px4_current_yaw_deg_normalized
                        self.last_static_yaw_deg = px4_current_yaw_deg_normalized
                        if self.last_static_yaw_deg < 0:
                            self.last_static_yaw_deg += 360
                        self.last_static_yaw_rad = px4_current_yaw_rad
                    
                    self.on_trajectory = True
                    self.trajectory_start_time = time.time()
                    self.command_goto_requested = False
                    
                    # GOTO sempre inicia em GIRANDO_INICIO (simplificação do fluxo)
                    self.mudar_estado(DroneStateDescription.VOANDO_GIRANDO_INICIO)
                    return
                
                # Verificar flag de goto_focus
                if cmd_goto_focus_requested:
                    self.on_trajectory = True
                    self.trajectory_start_time = time.time()
                    self.command_goto_focus_requested = False
                    # GOTO_FOCUS inicia com rotação para apontar ao focus
                    self.mudar_estado(DroneStateDescription.VOANDO_GIRANDO_COM_FOCO)
                    return
                
                # Verificar flag de land
                if cmd_land_requested:
                    self.on_trajectory = False  # Pouso é controlado pelo PX4
                    # Armazena posição estática para referência
                    if px4_local_position is not None:
                        self.last_static_position = [
                            px4_local_position.x,
                            px4_local_position.y,
                            px4_local_position.z
                        ]
                        # Armazena yaw estático (normalized = -180/180, deg = 0-360)
                        self.last_static_yaw_deg_normalized = px4_current_yaw_deg_normalized
                        self.last_static_yaw_deg = px4_current_yaw_deg_normalized
                        if self.last_static_yaw_deg < 0:
                            self.last_static_yaw_deg += 360
                        self.last_static_yaw_rad = px4_current_yaw_rad
                    self.command_land_requested = False
                    self.mudar_estado(DroneStateDescription.POUSANDO)
                    return
                
                # Verificar flag de RTL
                if cmd_rtl_requested:
                    self.on_trajectory = True
                    self.trajectory_start_time = time.time()
                    self.command_rtl_requested = False
                    self.mudar_estado(DroneStateDescription.RETORNANDO_GIRANDO_INICIO)
                    return
            
            case DroneStateDescription.VOANDO_DECOLANDO:
                
                # Verificar progresso da decolagem
                if px4_local_position is None or local_target_local_position is None:
                    return
                
                current_alt = -px4_local_position.z
                target_alt = -local_target_local_position[2]
                self.node.get_logger().info(
                    f"Decolagem: altitude atual={current_alt:.2f}m, alvo={target_alt:.2f}m",
                    throttle_duration_sec=1.0
                )
                
                if abs(current_alt - target_alt) <= self.position_tolerance:
                    self.node.get_logger().info(
                        f"Decolagem completa! Altitude atual: {current_alt:.2f}m"
                    )
                    self.reseta_variaveis_estado()
                    self.mudar_estado(DroneStateDescription.VOANDO_PRONTO)
            
            case DroneStateDescription.VOANDO_GIRANDO_INICIO:
                
                # Verificar se yaw de direção foi alcançado
                if px4_local_position is None or local_target_local_position is None:
                    return
                
                # Se não tem yaw de direção definido, avança imediatamente
                if local_target_direction_yaw_deg is None:
                    self.node.get_logger().info(
                        "GIRANDO_INICIO: Sem yaw de direção definido, avançando para VOANDO_A_CAMINHO."
                    )
                    self.mudar_estado(DroneStateDescription.VOANDO_A_CAMINHO)
                    return
                
                # Verifica diferença de yaw
                yaw_diff = px4_current_yaw_deg_normalized - local_target_direction_yaw_deg
                if yaw_diff < -180:
                    yaw_diff += 360
                if yaw_diff > 180:
                    yaw_diff -= 360
                
                # Se já está em período de estabilização, apenas verifica tempo
                if self.yaw_aligned_time is not None:
                    if (time.time() - self.yaw_aligned_time) >= self.yaw_stabilization_delay:
                        # Passou o tempo de estabilização - inicia movimento
                        self.yaw_aligned_time = None  # Reseta para próximo uso
                        self.node.get_logger().info(
                            f"Estabilização completa. Iniciando movimento."
                        )
                        self.mudar_estado(DroneStateDescription.VOANDO_A_CAMINHO)
                    else:
                        self.node.get_logger().info(
                            f"Aguardando estabilização...",
                            throttle_duration_sec=1.0
                        )
                    return

                if abs(yaw_diff) <= self.yaw_tolerance_deg:
                    # Yaw alinhado - inicia período de estabilização
                    self.yaw_aligned_time = time.time()
                    self.node.get_logger().info(
                        f"Yaw de direção alcançado ({local_target_direction_yaw_deg:.1f}°). Aguardando estabilização..."
                    )
            
            case DroneStateDescription.VOANDO_A_CAMINHO:
                
                # Verificar se chegou ao destino
                if px4_local_position is None or local_target_local_position is None:
                    return
                
                current_x = px4_local_position.x
                current_y = px4_local_position.y
                current_z = px4_local_position.z
                
                target_x, target_y, target_z = local_target_local_position
                
                dx = target_x - current_x
                dy = target_y - current_y
                dz = target_z - current_z
                distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)
                
                if distance_to_target <= self.position_tolerance:
                    # Chegou ao destino - armazena posição estática
                    if px4_local_position is not None:
                        self.last_static_position = [
                            px4_local_position.x,
                            px4_local_position.y,
                            px4_local_position.z
                        ]
                        self.last_static_yaw_deg_normalized = px4_current_yaw_deg_normalized
                        self.last_static_yaw_deg = px4_current_yaw_deg_normalized
                        if self.last_static_yaw_deg < 0:
                            self.last_static_yaw_deg += 360
                        self.last_static_yaw_rad = px4_current_yaw_rad
                    
                    # SEMPRE vai para GIRANDO_FIM (simplificação do fluxo)
                    yaw_info = f" (yaw alvo: {local_target_final_yaw_deg:.1f}°)" if local_target_final_yaw_deg else " (sem yaw final)"
                    self.node.get_logger().info(
                        f"Posição alvo alcançada. Avançando para GIRANDO_FIM{yaw_info}."
                    )
                    self.mudar_estado(DroneStateDescription.VOANDO_GIRANDO_FIM)
            
            case DroneStateDescription.VOANDO_GIRANDO_COM_FOCO:
                
                # Verificar se yaw para o focus foi alcançado
                if px4_local_position is None or local_focus_local_position is None:
                    return
                
                # Calcula yaw para apontar para o focus da posição atual
                focus_dx = local_focus_local_position[0] - px4_local_position.x
                focus_dy = local_focus_local_position[1] - px4_local_position.y
                focus_yaw = math.degrees(math.atan2(focus_dy, focus_dx))
                if focus_yaw > 180:
                    focus_yaw -= 360
                elif focus_yaw < -180:
                    focus_yaw += 360
                
                # Se já está em período de estabilização, apenas verifica tempo
                if self.yaw_aligned_time is not None:
                    if (time.time() - self.yaw_aligned_time) >= self.yaw_stabilization_delay:
                        # Passou o tempo de estabilização - inicia movimento
                        self.yaw_aligned_time = None  # Reseta para próximo uso
                        self.node.get_logger().info(
                            f"GOTO_FOCUS: Estabilização completa. Iniciando movimento."
                        )
                        self.mudar_estado(DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO)
                    else:
                        self.node.get_logger().info(
                            f"GOTO_FOCUS: Aguardando estabilização...",
                            throttle_duration_sec=1.0
                        )
                    return
                
                # Verifica se atingiu o yaw alvo
                yaw_diff = px4_current_yaw_deg_normalized - focus_yaw
                if yaw_diff < -180:
                    yaw_diff += 360
                if yaw_diff > 180:
                    yaw_diff -= 360
                
                if abs(yaw_diff) <= self.yaw_tolerance_deg:
                    # Yaw alinhado - inicia período de estabilização
                    self.yaw_aligned_time = time.time()
                    self.node.get_logger().info(
                        f"GOTO_FOCUS: Yaw para focus alcançado ({focus_yaw:.1f}°). Aguardando estabilização..."
                    )
            
            case DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO:
                
                # Verificar se chegou ao destino
                if px4_local_position is None or local_target_local_position is None:
                    return
                
                current_x = px4_local_position.x
                current_y = px4_local_position.y
                current_z = px4_local_position.z
                
                target_x, target_y, target_z = local_target_local_position
                
                dx = target_x - current_x
                dy = target_y - current_y
                dz = target_z - current_z
                distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)
                
                if distance_to_target <= self.position_tolerance:
                    # Se já está em período de estabilização, apenas verifica tempo
                    if self.yaw_aligned_time is not None:
                        if (time.time() - self.yaw_aligned_time) >= self.yaw_stabilization_delay:
                            # Passou o tempo de estabilização - trajetória completa
                            self.yaw_aligned_time = None  # Reseta para próximo uso
                            self.node.get_logger().info(
                                f"GOTO_FOCUS: Estabilização completa. Trajetória finalizada."
                            )
                            self.reseta_variaveis_estado()
                            self.mudar_estado(DroneStateDescription.VOANDO_PRONTO)
                        else:
                            self.node.get_logger().info(
                                f"GOTO_FOCUS: Chegou ao destino. Aguardando estabilização...",
                                throttle_duration_sec=1.0
                            )
                        return
                    
                    # Chegou ao destino - inicia período de estabilização
                    self.yaw_aligned_time = time.time()
                    self.node.get_logger().info(
                        f"GOTO_FOCUS: Posição alvo alcançada. Aguardando estabilização..."
                    )
            
            case DroneStateDescription.VOANDO_GIRANDO_FIM:
                
                # Verificar se yaw final foi alcançado
                if local_target_final_yaw_deg is None:
                    self.reseta_variaveis_estado()
                    self.mudar_estado(DroneStateDescription.VOANDO_PRONTO)
                    return
                
                # Se já está em período de estabilização, apenas verifica tempo
                if self.yaw_aligned_time is not None:
                    if (time.time() - self.yaw_aligned_time) >= self.yaw_stabilization_delay:
                        # Passou o tempo de estabilização - trajetória completa
                        self.yaw_aligned_time = None  # Reseta para próximo uso
                        self.node.get_logger().info(
                            f"Estabilização completa. Trajetória finalizada."
                        )
                        self.reseta_variaveis_estado()
                        self.mudar_estado(DroneStateDescription.VOANDO_PRONTO)
                    return
                
                yaw_diff = px4_current_yaw_deg_normalized - local_target_final_yaw_deg
                if yaw_diff < -180:
                    yaw_diff += 360
                if yaw_diff > 180:
                    yaw_diff -= 360
                
                if abs(yaw_diff) <= self.yaw_tolerance_deg:
                    # Yaw alvo alcançado - inicia período de estabilização
                    self.node.get_logger().info(
                        f"Yaw alvo alcançado ({local_target_final_yaw_deg:.1f}°). Aguardando estabilização..."
                    )
                    self.yaw_aligned_time = time.time()
            
            # ============== ESTADOS RTL (Return To Launch) ==============
            
            case DroneStateDescription.RETORNANDO_GIRANDO_INICIO:
                
                # Verificar se yaw de direção (para HOME) foi alcançado
                if local_target_direction_yaw_deg is None:
                    return
                
                # Se já está em período de estabilização, apenas verifica tempo
                if self.yaw_aligned_time is not None:
                    if (time.time() - self.yaw_aligned_time) >= self.yaw_stabilization_delay:
                        # Passou o tempo de estabilização - inicia movimento para HOME
                        self.yaw_aligned_time = None  # Reseta para próximo uso
                        self.node.get_logger().info(
                            f"RTL: Estabilização completa. Iniciando voo para HOME."
                        )
                        self.mudar_estado(DroneStateDescription.RETORNANDO_A_CAMINHO)
                    return
                
                # Verifica se atingiu o yaw alvo
                yaw_diff = px4_current_yaw_deg_normalized - local_target_direction_yaw_deg
                if yaw_diff < -180:
                    yaw_diff += 360
                if yaw_diff > 180:
                    yaw_diff -= 360
                
                if abs(yaw_diff) <= self.yaw_tolerance_deg:
                    # Yaw alinhado - inicia período de estabilização
                    self.yaw_aligned_time = time.time()
                    self.node.get_logger().info(
                        f"RTL: Yaw de direção alcançado ({local_target_direction_yaw_deg:.1f}°). Aguardando estabilização..."
                    )
            
            case DroneStateDescription.RETORNANDO_A_CAMINHO:
                
                # Verificar se chegou ao HOME
                if px4_local_position is None or local_target_local_position is None:
                    return
                
                current_x = px4_local_position.x
                current_y = px4_local_position.y
                current_z = px4_local_position.z
                
                target_x, target_y, target_z = local_target_local_position
                
                dx = target_x - current_x
                dy = target_y - current_y
                dz = target_z - current_z
                distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)
                
                if distance_to_target <= self.position_tolerance:
                    # Chegou ao HOME - Armazena posição estática para hover estável durante giro
                    if px4_local_position is not None:
                        self.last_static_position = [
                            px4_local_position.x,
                            px4_local_position.y,
                            px4_local_position.z
                        ]
                        self.last_static_yaw_deg_normalized = px4_current_yaw_deg_normalized
                        self.last_static_yaw_deg = px4_current_yaw_deg_normalized
                        if self.last_static_yaw_deg < 0:
                            self.last_static_yaw_deg += 360
                        self.last_static_yaw_rad = px4_current_yaw_rad
                    # Transiciona para rotação final
                    self.node.get_logger().info(
                        f"RTL: Posição HOME alcançada. Girando para yaw final ({self.px4.home_yaw_deg:.1f}°)..."
                    )
                    self.mudar_estado(DroneStateDescription.RETORNANDO_GIRANDO_FIM)
            
            case DroneStateDescription.RETORNANDO_GIRANDO_FIM:
                
                # Verificar se home_yaw_deg foi alcançado
                if self.px4.home_yaw_deg is None:
                    # Sem yaw final específico, vai direto para pouso
                    self.reseta_variaveis_estado()
                    self.mudar_estado(DroneStateDescription.POUSANDO)
                    self.node.land()  # Envia comando de pouso nativo
                    self.command_land_requested = False
                    self.node.get_logger().info("RTL: Iniciando pouso...")
                    return
                
                # Se já está em período de estabilização, apenas verifica tempo
                if self.yaw_aligned_time is not None:
                    if (time.time() - self.yaw_aligned_time) >= self.yaw_stabilization_delay:
                        # Passou o tempo de estabilização - inicia pouso
                        self.yaw_aligned_time = None  # Reseta para próximo uso
                        self.node.get_logger().info(
                            f"RTL: Estabilização completa. Iniciando pouso..."
                        )
                        self.reseta_variaveis_estado()
                        self.mudar_estado(DroneStateDescription.POUSANDO)
                        self.node.land()  # Envia comando de pouso nativo
                        self.command_land_requested = False # 
                    else:
                        self.node.get_logger().info(
                            f"RTL: Aguardando estabilização...",
                            throttle_duration_sec=1.0
                        )
                    return
                
                yaw_diff = px4_current_yaw_deg_normalized - self.px4.home_yaw_deg
                if yaw_diff < -180:
                    yaw_diff += 360
                if yaw_diff > 180:
                    yaw_diff -= 360
                
                if abs(yaw_diff) <= self.yaw_tolerance_deg:
                    # Yaw HOME alcançado - inicia período de estabilização
                    self.node.get_logger().info(
                        f"RTL: Yaw HOME alcançado ({self.px4.home_yaw_deg:.1f}°). Aguardando estabilização..."
                    )
                    self.yaw_aligned_time = time.time()
            
            case DroneStateDescription.POUSANDO:
                
                # Verificar se pousou
                if px4_is_landed:
                    self.node.get_logger().info("Pouso completo. Aguardando desarme automático do PX4...", throttle_duration_sec=5.0)
            
            case _:
                # Estado desconhecido - tenta recuperar para estado seguro
                self.node.get_logger().warn(f"Estado desconhecido: {current_state}. Tentando recuperar...")
                if px4_is_armed:
                    if not px4_is_landed and not self.on_trajectory:
                        self.mudar_estado(DroneStateDescription.VOANDO_PRONTO)
                else:
                    self.mudar_estado(DroneStateDescription.POUSADO_DESARMADO)
    


    def mudar_estado(self, new_state: DroneStateDescription):
        """
        Executa transição para novo estado e notifica a mudança.
        
        Args:
            new_state: Novo estado para transicionar
        """
        old_state = self.state
        if self.state != new_state:
            self.state = new_state
            self.state_entry_time = time.time()

            # Log da transição
            self.node.get_logger().info("-----------------------------------------------------------------------------------")
            self.node.get_logger().info(f">>> Mudança de Estado: {old_state.name} -> {self.state.name}")


    def verifica_condicao_de_emergencia(self) -> bool:
        """
        Verifica condições de emergência que requerem transição imediata.
        
        Returns:
            bool: True se há condição de emergência ativa
        """
        # Bateria crítica (menos de 10%)
        if self.px4.battery_status and self.px4.battery_status.remaining < 0.10:
            self.node.get_logger().warn("EMERGÊNCIA: Bateria crítica!")
            return True
        
        # Adicione outras condições de emergência aqui conforme necessário
        # Exemplo: perda de GPS, failsafe do PX4, etc.
        
        return False
    
    def verifica_validade_do_comando(self, command: str) -> tuple:
        """
        Verifica se um comando pode ser executado no estado atual.
        
        Args:
            command: Nome do comando (ARM, DISARM, TAKEOFF, GOTO, LAND, RTL)
        
        Returns:
            tuple: (pode_executar: bool, mensagem_erro: str)
        """
        
        if self.px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return False, "Drone NÃO está em modo OFFBOARD."

        if self.px4.local_position is None:
            return False, "Não é possível processar comando: Posição local desconhecida."
        
        allowed_states = self.VALID_COMMANDS.get(command, [])
        
        if not allowed_states:
            # Comando desconhecido - permite execução (será tratado pelo callback)
            return True, ""
        
        if self.state in allowed_states:
            return True, ""
        else:
            allowed_names = [s.name for s in allowed_states]
            return False, (
                f"Comando '{command}' não permitido no estado {self.state.name}. "
                f"Estados permitidos: {allowed_names}"
            )
    





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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        drone_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()




