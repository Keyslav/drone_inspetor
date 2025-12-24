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
from drone_inspetor_msgs.msg import DroneStateMSG, MissionCommandMSG

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
    PX4_RX_PREFIX = "\033[36m\033[1mMENSAGEM PX4 -\033[0m "
    PX4_TX_PREFIX = "\033[35m\033[1mCOMANDO PX4 -\033[0m "

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

        # --- Armazenamento de Estado Interno do Veículo ---
        self.PX4_vehicle_status = None        # Último status geral do drone (armado, modo, etc.)
        self.PX4_local_position = None        # Última posição local (X, Y, Z) em metros
        self.PX4_home_position = None         # Posição de home (ponto de partida)
        self.PX4_global_position = None       # Última posição global (latitude, longitude, altitude)
        self.PX4_vehicle_attitude = None      # Última atitude (orientação) do drone
        self.PX4_current_yaw_deg = 0.0        # Yaw atual do drone (em graus)
        self.PX4_current_yaw_rad = 0.0        # Yaw atual do drone (em radianos)
        self.PX4_current_yaw_deg_normalized = 0.0 # Yaw atual do drone (em graus) normalizado (0-360)
        self.PX4_current_control_mode = None   # Modo de controle atual do PX4
        self.PX4_is_armed = False             # Flag booleano para o estado de armamento
        self.is_arming = False                # Flag booleano para o estado de armamento
        self.is_disarming = False             # Flag booleano para processo de desarmamento
        self.PX4_is_landed = False            # Flag booleano para o estado de pouso
        # Indica se o drone está publicando setpoints de trajetória de movimento (moving),
        # ao invés de manter posição (static).
        self.on_trajectory = False
        self.PX4_last_command_ack = None      # Última confirmação de comando recebida do PX4
        self.offboard_setpoint_counter = 0    # Contador para garantir envio contínuo de setpoints
        self.PX4_offboard_mode_active = False # Flag para indicar se o modo Offboard está ativo
        self.trajectory_start_time = None     # Timestamp do início do movimento (em segundos)
        self.PX4_battery_status = None        # Status da bateria
        self.OffboardControlMode_internal_enabled = False  # Habilita publicação OffboardControlMode/TrajectorySetpoint

        # --- Parâmetros de Trajetória Simplificada ---
        self.max_velocity = 1.0               # Velocidade máxima (m/s)
        self.step_distance = 3.0              # Distância do próximo passo (metros)

        # --- Parâmetros de Navegação ---
        self.origin_local_position = None     # Posição de origem quando inicia trajetória [x, y, z]
        self.target_local_position = None     # Posição alvo para trajetória [x, y, z]
        self.target_latitude = None           # Latitude alvo para trajetória (graus)
        self.target_longitude = None          # Longitude alvo para trajetória (graus)
        self.target_altitude = None           # Altitude alvo para trajetória (metros)
        self.origin_latitude = None           # Latitude de origem quando inicia trajetória (graus)
        self.origin_longitude = None          # Longitude de origem quando inicia trajetória (graus)
        self.origin_altitude = None           # Altitude de origem quando inicia trajetória (metros)
        self.target_yaw = None                # Yaw alvo final (graus, 0-360) - None significa manter yaw atual
        self.position_tolerance = 0.5         # Tolerância em metros para considerar que chegou ao destino
        self.yaw_tolerance_deg = 5.0          # Tolerância em graus para o yaw (apontamento)
        
        # --- Estados da Trajetória ---
        # A fase da trajetória agora é representada diretamente por DroneStateDescription (state machine)
        self.direction_yaw = None             # Yaw necessário para apontar na direção do destino (graus)
        self.takeoff_altitude = 2.5           # Altitude padrão de decolagem (metros)
        self.rtl_altitude = 30.0              # Altitude do RTL em metros (sobe antes de ir para home)
        self.rtl_auto_disarm = False          # Se True, desarma automaticamente após pousar (usado pelo RTL)
        self.rtl_pending_land = False         # Se True, após chegar ao destino transiciona para VOANDO_POUSANDO

        # ==================================================================
        # SUBSCRIBERS (Ouvindo o PX4 e a FSM)
        # ==================================================================

        # --- Tópicos de Telemetria do PX4 (/fmu/out/...) ---
        # Estes tópicos fornecem informações sobre o estado atual do drone
        
        # Status e controle do veículo
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.vehicle_status_callback, qos_profile)
        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.vehicle_control_mode_callback, qos_profile)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.command_ack_callback, qos_profile)
        # self.create_subscription(ModeCompleted, "/fmu/out/mode_completed", self.mode_completed_callback, qos_profile)
        
        # Posição e navegação
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_position_callback, qos_profile)
        self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_position_callback, qos_profile)
        # self.create_subscription(VehicleGpsPosition, "/fmu/out/vehicle_gps_position", self.vehicle_gps_position_callback, qos_profile)
        # self.create_subscription(HomePosition, "/fmu/out/home_position", self.home_position_callback, qos_profile)
        
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

        # --- Tópico de Comando da FSM ---
        # Este é o canal principal de comunicação com a FSM (usando MissionCommandMSG)
        self.fsm_command_sub = self.create_subscription(
            MissionCommandMSG, 
            "/drone_inspetor/interno/fsm_node/drone_commands", 
            self.fsm_command_callback, 
            qos_profile
        )

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
        self.goto_setpoint_pub = self.create_publisher(GotoSetpoint, "/fmu/in/goto_setpoint", qos_profile)
        
        # Setpoints de atitude e taxas
        self.vehicle_attitude_setpoint_pub = self.create_publisher(VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint", qos_profile)
        self.vehicle_rates_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos_profile)
        
        # Setpoints de força e torque
        self.vehicle_thrust_setpoint_pub = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", qos_profile)
        self.vehicle_torque_setpoint_pub = self.create_publisher(VehicleTorqueSetpoint, "/fmu/in/vehicle_torque_setpoint", qos_profile)
        
        # Sensores externos
        self.distance_sensor_pub = self.create_publisher(DistanceSensor, "/fmu/in/distance_sensor", qos_profile)
        self.obstacle_distance_pub = self.create_publisher(ObstacleDistance, "/fmu/in/obstacle_distance", qos_profile)
        self.sensor_optical_flow_pub = self.create_publisher(SensorOpticalFlow, "/fmu/in/sensor_optical_flow", qos_profile)
        
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
        self.state_machine = DroneState(self)

        # ==================================================================
        # TIMERS (Executando ações periódicas)
        # ==================================================================

        # Timers separados (0.1s): OffboardControlMode e TrajectorySetpoint
        self.offboard_control_mode_timer = self.create_timer(0.1, self.timer_offboard_control_mode)
        self.trajectory_setpoint_timer = self.create_timer(0.1, self.timer_trajectory_setpoint)

        # Timer para publicação periódica de status JSON (a cada 0.5 segundos)
        self.status_pub_timer = self.create_timer(0.5, lambda: self.publish_drone_status())

        # Timer para atualização da máquina de estados (a cada 0.5 segundos)
        self.state_machine_timer = self.create_timer(0.5, self.update_state_machine)

        self.get_logger().info("================ DRONE NODE PRONTO ================")

    # ==================================================================
    # SEÇÃO 1: TIMERS - Funções Principais (executadas periodicamente)
    # ==================================================================

    def update_state_machine(self):
        """
        Atualiza a máquina de estados periodicamente (a cada 0.5s).
        Este método é chamado pelo timer state_machine_timer.
        """
        old_state = self.state_machine.get_state()
        self.state_machine._check_transitions()
        if old_state != self.state_machine.get_state():
            self.state_machine._on_state_changed(old_state)

    def timer_offboard_control_mode(self):
        """Publica OffboardControlMode a cada 0.1s (se habilitado)."""
        if not self.OffboardControlMode_internal_enabled:
            return
        self.publish_offboard_control_mode()

    def timer_trajectory_setpoint(self):
        """Publica TrajectorySetpoint a cada 0.1s (se habilitado)."""
        if not self.OffboardControlMode_internal_enabled:
            return

        # Validação: precisa ter posição local e global para enviar setpoints
        if self.PX4_global_position is None or self.PX4_local_position is None:
            self.get_logger().error(f"{self.PX4_RX_PREFIX}Não é possível publicar setpoints Offboard: Posição local desconhecida.")
            return

        # Decide qual tipo de setpoint enviar baseado em on_trajectory (controlada pelos comandos/transições)
        if self.on_trajectory and self.target_local_position is not None:
            trajectory_msg = self.create_moving_trajectory_setpoint()
        else:
            # Nenhum movimento: mantém posição atual (hover)
            trajectory_msg = self.create_static_position_setpoint()
        
        # Publica o setpoint
        self.trajectory_setpoint_pub.publish(trajectory_msg)

        # Log dos valores do trajectory publicado
#        self.get_logger().info(
#            f"Trajectory Setpoint publicado - "
#            f"\nPosição: [{trajectory_msg.position[0]:.3f}, {trajectory_msg.position[1]:.3f}, {trajectory_msg.position[2]:.3f}] | "
#            f"\nYaw: {trajectory_msg.yaw if not math.isnan(trajectory_msg.yaw) else 'NaN'}",
#            throttle_duration_sec=2.0
#        )


    def publish_offboard_control_mode(self):
        """
        Publica o modo de controle Offboard habilitando apenas posição.
        """
        if not self.OffboardControlMode_internal_enabled:
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
        
        # Status de controle
        msg.offboard_mode_active = self.PX4_offboard_mode_active
        msg.is_armed = self.PX4_is_armed
        msg.is_arming = self.is_arming
        msg.is_landed = self.PX4_is_landed
        msg.on_trajectory = self.on_trajectory
        
        # Posição local (NED)
        if self.PX4_local_position:
            msg.local_x = self.PX4_local_position.x
            msg.local_y = self.PX4_local_position.y
            msg.local_z = -self.PX4_local_position.z  # Z é negativo, então invertemos
        else:
            msg.local_x = 0.0
            msg.local_y = 0.0
            msg.local_z = 0.0
        
        # Posição global (GPS)
        if self.PX4_global_position:
            msg.latitude = self.PX4_global_position.lat
            msg.longitude = self.PX4_global_position.lon
            msg.altitude = self.PX4_global_position.alt
        else:
            msg.latitude = 0.0
            msg.longitude = 0.0
            msg.altitude = 0.0
        
        # Orientação
        msg.yaw_deg = self.PX4_current_yaw_deg
        msg.yaw_rad = self.PX4_current_yaw_rad
        
        # Estado da máquina de estados do drone (sem sub_state_name)
        state_dict = self.state_machine.get_status_dict()
        msg.state_name = state_dict.get("drone_state", "POUSADO_DESARMADO")
        msg.state_duration_sec = state_dict.get("state_duration_sec", 0.0)
        
        # Publica mensagem
        self.drone_state_pub.publish(msg)
        

    def create_static_position_setpoint(self):
        """
        Cria um setpoint de trajetória para manter a posição atual (hover).
        
        Returns:
            TrajectorySetpoint: Mensagem com posição atual e velocidades zeradas
        """
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Mantém a posição atual
        trajectory_msg.position[0] = self.PX4_local_position.x
        trajectory_msg.position[1] = self.PX4_local_position.y
        trajectory_msg.position[2] = self.PX4_local_position.z
        
        # Deixa o PX4 controlar o yaw por padrão
        trajectory_msg.yaw = float('nan')
        
        return trajectory_msg

    def create_moving_trajectory_setpoint(self):
        """
        Cria um setpoint de trajetória simplificada calculando o próximo ponto a 1 metro
        em direção ao destino, baseado apenas na posição atual e no destino.
        Implementa três fases: rotação inicial, movimento, rotação final.
        
        Returns:
            TrajectorySetpoint: Mensagem com setpoint completo
        """
        # Calcula posição e yaw na trajetória simplificada
        next_x, next_y, next_z, target_yaw = self.calculate_next_trajectory_by_state()
        
        # Cria mensagem de trajetória
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Posição: próximo ponto a 1 metro em direção ao destino
        trajectory_msg.position[0] = next_x
        trajectory_msg.position[1] = next_y
        trajectory_msg.position[2] = next_z
        
        # Define o yaw alvo (em radianos)
        if target_yaw is not None:
            trajectory_msg.yaw = math.radians(target_yaw)
        else:
            # Deixa o PX4 controlar o yaw por padrão
            trajectory_msg.yaw = float('nan')
        
        return trajectory_msg

    # ==================================================================
    # SEÇÃO 2: CÁLCULO DE TRAJETÓRIA - Funções Auxiliares
    # ==================================================================

    def calculate_next_trajectory_by_state(self):
        """
        Calcula o próximo ponto da trajetória baseado na posição atual e no destino.
        Implementa quatro fases sequenciais:
        0. TAKEOFF: Sobe até a altitude de decolagem (apenas para decolagem)
        1. ROTATING_TO_DIRECTION: Rotaciona o drone para apontar na direção do destino
        2. MOVING: Move o drone até a posição alvo
        3. ROTATING_TO_TARGET_YAW: Rotaciona o drone para o yaw alvo final (se especificado)
        
        Returns:
            Tupla (x, y, z, yaw) onde:
            - x, y, z: Próxima posição em coordenadas locais (metros, frame NED)
            - yaw: Yaw alvo em graus (0-360) ou None se não precisa controlar yaw
        """
        # Obtém posição atual
        current_x = self.PX4_local_position.x
        current_y = self.PX4_local_position.y
        current_z = self.PX4_local_position.z
        
        # Obtém posição alvo
        target_x = self.target_local_position[0]
        target_y = self.target_local_position[1]
        target_z = self.target_local_position[2]
        
        # Calcula o vetor de deslocamento da posição atual para o destino
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        
        # Calcula a distância total até o destino
        distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Estado atual (fonte única de verdade para fases)
        current_state = self.state_machine.get_state()

        # FASE 0: TAKEOFF - Subindo até altitude de decolagem
        if current_state == DroneStateDescription.VOANDO_DECOLANDO:
            # Calcula altitude atual (Z negativo = para cima no frame NED)
            current_alt = -current_z
            target_alt = -target_z

            # Ainda subindo - mantém posição X,Y e vai para Z alvo
            self.get_logger().info(
                f"TAKEOFF: altitude atual={current_alt:.2f}m, alvo={target_alt:.2f}m",
                throttle_duration_sec=1.0
            )
            return (current_x, current_y, target_z, self.direction_yaw)
        
        # FASE 1: ROTATING_TO_DIRECTION - Rotaciona para apontar na direção do destino
        if current_state == DroneStateDescription.VOANDO_GIRANDO_INICIO:
            # Apenas ajusta yaw, transições são tratadas em _check_transitions()
            return (current_x, current_y, current_z, self.direction_yaw)

        # FASE 2: MOVING - Move até a posição alvo
        if current_state == DroneStateDescription.VOANDO_A_CAMINHO:
            # Pode acontecer do estado ainda não ter sido atualizado (timer de 0.5s),
            # mas a posição já estar exatamente no alvo (distância zero). Evita divisão por zero.
            if distance_to_target <= 1e-6:
                return (target_x, target_y, target_z, self.direction_yaw)

            # Normaliza o vetor de direção
            direction = [dx / distance_to_target, dy / distance_to_target, dz / distance_to_target]
            
            # Calcula a distância do próximo passo (1 metro ou a distância restante, o que for menor)
            step_dist = min(self.step_distance, distance_to_target)
            
            # Calcula o próximo ponto a 1 metro (ou menos) em direção ao destino
            next_x = current_x + direction[0] * step_dist
            next_y = current_y + direction[1] * step_dist
            next_z = current_z + direction[2] * step_dist
            
            # Durante o movimento, mantém o yaw de direção
            return (next_x, next_y, next_z, self.direction_yaw)
        
        # FASE 3: ROTATING_TO_TARGET_YAW - Rotaciona para o yaw alvo final
        if current_state == DroneStateDescription.VOANDO_GIRANDO_FIM:
            # Apenas ajusta yaw, transições são tratadas em _check_transitions()
            return (target_x, target_y, target_z, self.target_yaw)
        
        # FASE 4: VOANDO_POUSANDO - Move até home e pousa
        if current_state == DroneStateDescription.VOANDO_POUSANDO:
            # Evita divisão por zero se já está no alvo
            if distance_to_target <= 1e-6:
                return (target_x, target_y, target_z, self.direction_yaw)
            
            # Normaliza o vetor de direção
            direction = [dx / distance_to_target, dy / distance_to_target, dz / distance_to_target]
            
            # Calcula a distância do próximo passo
            step_dist = min(self.step_distance, distance_to_target)
            
            # Calcula o próximo ponto em direção ao destino (home)
            next_x = current_x + direction[0] * step_dist
            next_y = current_y + direction[1] * step_dist
            next_z = current_z + direction[2] * step_dist
            
            return (next_x, next_y, next_z, self.direction_yaw)
        
        # Estado desconhecido - mantém posição alvo
        return (target_x, target_y, target_z, None)


    def global_to_local_position(self, target_lat, target_lon, target_alt):
        """
        Converte coordenadas globais (GPS) para coordenadas locais (NED) relativas à posição atual.
        
        Args:
            target_lat: Latitude alvo (graus)
            target_lon: Longitude alvo (graus)
            target_alt: Altitude alvo (metros)
        
        Returns:
            Lista [x, y, z] com posição local em metros (coordenadas NED)
            None se não houver posição global atual disponível
        """
        if self.PX4_global_position is None or self.PX4_local_position is None:
            return None
        
        # Calcula offset da posição global atual para a posição alvo global
        offset = self.global_to_local_offset(
            self.PX4_global_position.lat,
            self.PX4_global_position.lon,
            self.PX4_global_position.alt,
            target_lat,
            target_lon,
            target_alt
        )
        
        # Aplica offset à posição local atual
        # No frame NED: X=norte, Y=leste, Z=para baixo (negativo = para cima)
        local_x = self.PX4_local_position.x + offset[0]  # Norte
        local_y = self.PX4_local_position.y + offset[1]   # Leste
        local_z = self.PX4_local_position.z - offset[2]  # Para cima (inverte porque Z é negativo para cima)
        
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
    

    def stop_movement(self):
        """
        Para a trajetória e limpa todas as variáveis relacionadas.
        """
        self.on_trajectory = False
        self.target_latitude = None
        self.target_longitude = None
        self.target_altitude = None
        self.origin_latitude = None
        self.origin_longitude = None
        self.origin_altitude = None
        self.origin_local_position = None
        self.target_local_position = None
        self.target_yaw = None
        self.direction_yaw = None
        self.trajectory_start_time = None
        self.rtl_pending_land = False

    def fsm_command_callback(self, msg: MissionCommandMSG):
        """
        Processa comandos de alto nível recebidos da FSM via mensagem MissionCommandMSG.
        Este é o callback principal que traduz comandos da FSM em ações do PX4.
        
        COMANDOS SUPORTADOS (MissionCommandMSG):
        - command="ARM": Arma os motores
        - command="DISARM": Desarma os motores
        - command="TAKEOFF", altitude=3.0: Decola até a altitude especificada
        - command="LAND": Pousa na posição atual
        - command="GOTO", lat=..., lon=..., alt=..., yaw=...: Move o drone
        - command="RTL": Retorna para casa
        - command="ENABLE_OFFBOARD": Habilita publicação OffboardControlMode/TrajectorySetpoint
        """
        command = msg.command
        
        self.get_logger().info(f"--> Comando da FSM recebido (MissionCommandMSG): {command}")

        if self.PX4_local_position is None:
            self.get_logger().error("Não é possível processar comando: Posição local desconhecida.")
            return

        # Valida se o comando é permitido no estado atual
        can_execute, error_msg = self.state_machine.can_execute_command(command)
        if not can_execute:
            self.get_logger().warn(f"Comando rejeitado: {error_msg}")
            return

        # Processa comandos usando match/case (Python 3.10+)
        match command:
            case "ARM":
                self.arm()
            case "DISARM":
                self.disarm()
            case "ENABLE_OFFBOARD":
                if not self.OffboardControlMode_internal_enabled:
                    self.get_logger().info("OffboardControlMode_internal_enabled habilitado via comando da FSM")
                self.OffboardControlMode_internal_enabled = True
            case "TAKEOFF":
                # Extrai altitude da mensagem (usa padrão se não especificado ou zero)
                alt = msg.altitude if msg.altitude > 0.0 else self.takeoff_altitude
                self.takeoff(alt)
            case "GOTO":
                # Extrai coordenadas da mensagem
                lat = msg.lat if msg.lat != 0.0 else None
                lon = msg.lon if msg.lon != 0.0 else None
                alt = msg.alt if msg.alt != 0.0 else None
                yaw = msg.yaw if msg.yaw != 0.0 else None  # Opcional
                self.goto(lat=lat, lon=lon, alt=alt, yaw=yaw)
            case "LAND":
                self.land()
            case "RTL":
                self.return_to_launch()
            case _:
                self.get_logger().warn(f"Comando não reconhecido: {command}")


    # ==================================================================
    # SEÇÃO 3: CALLBACKS - Funções Principais (processando telemetria)
    # ==================================================================

    def vehicle_status_callback(self, msg):
        """
        Processa a mensagem de status do veículo vinda do PX4.
        Esta é uma das mensagens mais importantes, pois contém informações sobre:
        - Estado de armamento (armado/desarmado)
        - Modo de voo atual (manual, auto, offboard, etc.)
        - Estado de navegação (decolando, voando, pousando, etc.)
        - Flags de sistema e segurança
        """
        self.PX4_vehicle_status = msg
        
        # Atualiza o estado de armamento
        self.PX4_is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        if self.PX4_is_armed:
            self.is_arming = False
        else:
            self.is_disarming = False
            
        # Atualiza o estado de navegação
        self.PX4_nav_state = msg.nav_state
        
        # ==================== MONITORAMENTO DE ESTADOS DE NAVEGAÇÃO ====================
        # Monitora todos os estados de navegação possíveis do PX4 para detectar eventos importantes
        
        # ==================== ESTADO OFFBOARD ====================
        # Controle externo via ROS2 (usado pelo drone_node)
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Drone está em modo offboard: controle externo via ROS2
            # Este é o modo principal usado pelo drone_node para controle autônomo
            if not self.PX4_offboard_mode_active:
                self.PX4_offboard_mode_active = True
                self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: OFFBOARD (Modo Offboard Ativado)", throttle_duration_sec=2)
            pass
        else:
            # Detecta saída do modo offboard apenas quando há mudança de estado
            if self.PX4_offboard_mode_active:
                self.PX4_offboard_mode_active = False
                self.OffboardControlMode_internal_enabled = False
                self.get_logger().warn(f"{self.PX4_RX_PREFIX}Estado de navegação: Modo Offboard Desativado", throttle_duration_sec=2)
        
        return

        # ==================== ESTADOS MANUAIS ====================
        # Controle manual direto pelo piloto (sem assistência de estabilização)
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_MANUAL:
            # Modo manual: controle direto dos motores pelo piloto
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: MANUAL", throttle_duration_sec=2)
            pass
        
        # Controle manual com estabilização de atitude
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_STAB:
            # Modo de estabilização: mantém atitude nivelada, mas permite controle manual
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: STAB (Estabilização)", throttle_duration_sec=2)
            pass
        
        # Controle acrobático: permite manobras acrobáticas sem estabilização
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_ACRO:
            # Modo acrobático: controle direto das taxas de rotação
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: ACRO (Acrobático)", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADOS ASSISTIDOS ====================
        # Controle manual com assistência de altitude
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_ALTCTL:
            # Modo de controle de altitude: mantém altitude fixa, permite controle manual horizontal
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: ALTCTL (Controle de Altitude)", throttle_duration_sec=2)
            pass
        
        # Controle manual com assistência de posição
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
            # Modo de controle de posição: mantém posição fixa quando não há input do piloto
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: POSCTL (Controle de Posição)", throttle_duration_sec=2)
            pass
        
        # Controle de posição em velocidade reduzida (para voos próximos a objetos)
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_POSITION_SLOW:
            # Modo de posição lenta: permite voo mais preciso e controlado
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: POSITION_SLOW (Posição Lenta)", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADOS AUTOMÁTICOS ====================
        # Decolagem automática
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            # Drone está em processo de decolagem automática
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: AUTO_TAKEOFF (Decolagem Automática)", throttle_duration_sec=2)
            pass
        
        # Missão automática
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
            # Drone está executando uma missão automática pré-programada
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: AUTO_MISSION (Missão Automática)", throttle_duration_sec=2)
            pass
        
        # Retorno automático ao ponto de partida (RTL - Return To Launch)
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            # Drone está retornando automaticamente para o ponto de partida
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: AUTO_RTL (Retorno ao Ponto de Partida)", throttle_duration_sec=2)
            pass
        
        # Pouso automático
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # Drone está pousando automaticamente
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: AUTO_LAND (Pouso Automático)", throttle_duration_sec=2)
            pass
        
        # Loiter automático (voar em círculos mantendo posição)
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            # Drone está em modo loiter: voa em círculos mantendo posição fixa
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: AUTO_LOITER (Loiter Automático)", throttle_duration_sec=2)
            pass
        
        # Seguir alvo automaticamente
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
            # Drone está seguindo um alvo automaticamente
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: AUTO_FOLLOW_TARGET (Seguir Alvo)", throttle_duration_sec=2)
            pass
        
        # Órbita automática ao redor de um ponto
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_ORBIT:
            # Drone está orbitando ao redor de um ponto específico
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: ORBIT (Órbita)", throttle_duration_sec=2)
            pass
        
        # Pouso preciso automático (usando sensores de precisão)
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND:
            # Drone está em modo de pouso preciso usando sensores de precisão
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: AUTO_PRECLAND (Pouso Preciso)", throttle_duration_sec=2)
            pass
        
        # Decolagem automática VTOL (para veículos VTOL)
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
            # Veículo VTOL está em processo de decolagem automática
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: AUTO_VTOL_TAKEOFF (Decolagem VTOL)", throttle_duration_sec=2)
            pass
        
        # Descida controlada
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_DESCEND:
            # Drone está em descida controlada
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: DESCEND (Descida)", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADOS EXTERNOS (CUSTOMIZADOS) ====================
        # Estados externos permitem controle customizado por aplicações externas
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL1:
            # Estado externo 1: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: EXTERNAL1", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL2:
            # Estado externo 2: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: EXTERNAL2", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL3:
            # Estado externo 3: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: EXTERNAL3", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL4:
            # Estado externo 4: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: EXTERNAL4", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL5:
            # Estado externo 5: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: EXTERNAL5", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL6:
            # Estado externo 6: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: EXTERNAL6", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL7:
            # Estado externo 7: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: EXTERNAL7", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_EXTERNAL8:
            # Estado externo 8: modo customizado para aplicações externas
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: EXTERNAL8", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADOS LIVRES (RESERVADOS) ====================
        # Estados livres reservados para uso futuro ou customização
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_FREE1:
            # Estado livre 1: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: FREE1", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_FREE2:
            # Estado livre 2: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: FREE2", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_FREE3:
            # Estado livre 3: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: FREE3", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_FREE4:
            # Estado livre 4: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: FREE4", throttle_duration_sec=2)
            pass
        
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_FREE5:
            # Estado livre 5: reservado para uso futuro
            self.get_logger().info(f"{self.PX4_RX_PREFIX}Estado de navegação: FREE5", throttle_duration_sec=2)
            pass
        
        # ==================== ESTADO DE TERMINAÇÃO ====================
        # Estado de emergência: sistema está sendo terminado
        if self.PX4_nav_state == VehicleStatus.NAVIGATION_STATE_TERMINATION:
            # Estado de terminação: sistema está sendo desligado por segurança
            self.get_logger().warn(f"{self.PX4_RX_PREFIX}Estado de navegação: TERMINATION (Terminação de Emergência)", throttle_duration_sec=2)
            pass
        
        # ==================== VERIFICAÇÃO DE ESTADO DESCONHECIDO ====================
        # Verifica se o estado está dentro do range válido
        if self.PX4_nav_state >= VehicleStatus.NAVIGATION_STATE_MAX:
            # Estado fora do range válido: possível erro ou versão incompatível do PX4
            self.get_logger().error(f"{self.PX4_RX_PREFIX}Estado de navegação inválido ou desconhecido: {self.PX4_nav_state}", throttle_duration_sec=2)
            pass

    def vehicle_local_position_callback(self, msg):
        """
        Processa a mensagem de posição local do drone.
        A posição local é relativa ao ponto de partida (home) e é dada em metros.
        Coordenadas: X (norte), Y (leste), Z (para baixo, negativo = para cima)
        """
        self.PX4_local_position = msg
        

    def vehicle_global_position_callback(self, msg):
        """
        Processa a mensagem de posição global (GPS) do drone.
        Contém latitude, longitude e altitude em coordenadas globais.
        """
        self.PX4_global_position = msg
        

    def vehicle_attitude_callback(self, msg):
        """
        Processa a mensagem de atitude (orientação) do drone.
        A atitude é representada como um quaternião (q[0]=w, q[1]=x, q[2]=y, q[3]=z).
        """
        self.PX4_vehicle_attitude = msg
        
        # Calcula e publica o yaw atual em graus
        q_w = msg.q[0]
        q_x = msg.q[1]
        q_y = msg.q[2]
        q_z = msg.q[3]
        
        # Conversão de quaternião para ângulo de Euler (yaw)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        
        self.PX4_current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.PX4_current_yaw_deg = math.degrees(self.PX4_current_yaw_rad)
        # Normaliza para 0-360 graus
        self.PX4_current_yaw_deg_normalized = (self.PX4_current_yaw_deg + 360) % 360
        

    def land_detected_callback(self, msg):
        """
        Processa a mensagem de detecção de pouso.
        Indica se o drone detectou que pousou (através de sensores de pressão, acelerômetros, etc.)
        """
        self.PX4_is_landed = msg.landed


    def command_ack_callback(self, msg):
        """
        Processa confirmações de comandos enviados ao PX4.
        Cada comando enviado via VehicleCommand recebe uma confirmação indicando se foi aceito ou rejeitado.
        """
        self.PX4_last_command_ack = msg
        result_text = "ACEITO" if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED else "REJEITADO"
        self.get_logger().info(f"{self.PX4_RX_PREFIX}ACK Comando {msg.command}: {result_text} (código: {msg.result})")


    def vehicle_control_mode_callback(self, msg):
        """
        Processa modo de controle atual do veículo (VehicleControlMode).
        Usado para verificar se o drone está no modo Offboard.
        """
        self.PX4_current_control_mode = msg
        self.PX4_offboard_mode_active = msg.flag_control_offboard_enabled

        # Habilita automaticamente o envio interno quando entrar em Offboard por qualquer razão
        if self.PX4_offboard_mode_active and not self.OffboardControlMode_internal_enabled:
            self.OffboardControlMode_internal_enabled = True
            self.get_logger().info(f"{self.PX4_RX_PREFIX}OffboardControlMode_internal_enabled habilitado (modo Offboard ativo)")


    def home_position_callback(self, msg):
        """
        Processa posição de home (HomePosition).
        """
        self.PX4_home_position = msg # Armazena a posição home


    def battery_status_callback(self, msg):
        """
        Processa status da bateria (BatteryStatus)."""

        self.PX4_battery_status = msg
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
        self.get_logger().debug(
            f"{self.PX4_TX_PREFIX}[{self.PX4_TOPIC_VEHICLE_COMMAND}] "
            f"Comando (ID: {command}) publicado com parâmetros: {params}"
        )

    def arm(self):
        """
        Envia o comando para armar os motores do drone.
        Comando: VEHICLE_CMD_COMPONENT_ARM_DISARM com param1=1.0
        """
        if not self.is_arming:
            self.is_arming = True
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
        self.is_disarming = True
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
            altitude = self.takeoff_altitude
        
        self.get_logger().info(
            f"{self.PX4_TX_PREFIX}[{self.PX4_TOPIC_VEHICLE_COMMAND}] "
            f"Iniciando TAKEOFF para altitude de {altitude}m..."
        )
        
        # Configura trajetória de decolagem (vertical)
        self.origin_local_position = [
            self.PX4_local_position.x, 
            self.PX4_local_position.y, 
            self.PX4_local_position.z
        ]
        
        # Z negativo = para cima no frame NED
        self.target_local_position = [
            self.PX4_local_position.x, 
            self.PX4_local_position.y, 
            -altitude
        ]
        
        # Mantém yaw atual durante decolagem
        self.target_yaw = None
        self.direction_yaw = self.PX4_current_yaw_deg_normalized
        
        # Define fase de trajetória como TAKEOFF
        self.state_machine._transition_to(DroneStateDescription.VOANDO_DECOLANDO)
        self.on_trajectory = True
        self.trajectory_start_time = time.time()
        
        self.get_logger().info(
            f"TAKEOFF configurado: posição atual Z={-self.PX4_local_position.z:.2f}m, "
            f"altitude alvo={altitude}m"
        )

    def goto(self, lat=None, lon=None, alt=None, yaw=None):
        """
        Move o drone para uma posição específica usando coordenadas globais.
        Converte as coordenadas globais para locais e armazena para uso nos cálculos de trajetória.
        O drone deve estar armado e em modo Offboard para que isso funcione.
        
        Sequência de execução:
        1. Rotaciona o drone para apontar na direção do destino
        2. Move o drone até a posição alvo
        3. Rotaciona o drone para o yaw alvo (se especificado)
        
        Args:
            lat: Latitude alvo (graus) - obrigatório
            lon: Longitude alvo (graus) - obrigatório
            alt: Altitude alvo (metros) - obrigatório
            yaw: Yaw alvo final (graus, 0-360) - opcional, None mantém o yaw atual
        """
        if lat is None or lon is None or alt is None:
            self.get_logger().error("Não é possível executar GOTO: Coordenadas incompletas (lat, lon, alt são obrigatórias).")
            return
        
        # Armazena coordenadas globais para referência e verificação de chegada
        self.origin_latitude = self.PX4_global_position.lat
        self.origin_longitude = self.PX4_global_position.lon
        self.origin_altitude = self.PX4_global_position.alt
        self.target_latitude = lat
        self.target_longitude = lon
        self.target_altitude = alt
        self.target_yaw = yaw  # Pode ser None
        
        # Converte coordenadas globais para locais e armazena para cálculos de trajetória
        self.origin_local_position = [self.PX4_local_position.x, self.PX4_local_position.y, self.PX4_local_position.z]
        
        target_local = self.global_to_local_position(lat, lon, alt)
        if target_local is None:
            self.get_logger().error("Não é possível executar GOTO: Erro ao converter coordenadas globais para locais.")
            return
        
        self.target_local_position = target_local
        
        # Calcula o yaw necessário para apontar na direção do destino
        dx = target_local[0] - self.PX4_local_position.x
        dy = target_local[1] - self.PX4_local_position.y
        self.direction_yaw = math.degrees(math.atan2(dy, dx))
        self.direction_yaw = (self.direction_yaw + 360) % 360  # Normaliza para 0-360
        
        # Verifica se já está muito próximo do destino
        distance_to_target = math.sqrt(dx**2 + dy**2 + (target_local[2] - self.PX4_local_position.z)**2)
        
        if distance_to_target <= self.position_tolerance:
            # Já está na posição alvo, pula direto para rotação final ou completa
            if self.target_yaw is not None:
                self.state_machine._transition_to(DroneStateDescription.VOANDO_GIRANDO_FIM)
                self.get_logger().info(f"Já está próximo do destino. Iniciando rotação para yaw alvo ({self.target_yaw:.1f}°).")
            else:
                self.on_trajectory = False
                self.state_machine._transition_to(DroneStateDescription.VOANDO_PRONTO)
                self.get_logger().info("Já está próximo do destino. Trajetória completa.")
        elif self.PX4_is_landed:
            # Se o drone está pousado, não pode rotacionar - pula direto para movimento
            self.state_machine._transition_to(DroneStateDescription.VOANDO_A_CAMINHO)
            self.get_logger().info("Drone está pousado. Pulando rotação inicial e iniciando movimento direto.")
        else:
            # Inicia na fase de rotação para direção
            self.state_machine._transition_to(DroneStateDescription.VOANDO_GIRANDO_INICIO)
        
        # Marca que o drone está em trajetória e salva o momento inicial
        if self.state_machine.get_state() in (
            DroneStateDescription.VOANDO_GIRANDO_INICIO,
            DroneStateDescription.VOANDO_A_CAMINHO,
            DroneStateDescription.VOANDO_GIRANDO_FIM,
        ):
            self.on_trajectory = True
        self.trajectory_start_time = time.time()
        
        yaw_info = f", yaw_alvo={yaw:.1f}°" if yaw is not None else ", yaw_alvo=None (mantém atual)"
        self.get_logger().info(f"GOTO iniciado: origem global=[{self.origin_latitude:.6f}, {self.origin_longitude:.6f}, {self.origin_altitude:.2f}], alvo global=[{lat:.6f}, {lon:.6f}, {alt:.2f}]{yaw_info}")
        self.get_logger().info(f"GOTO: origem local=[{self.origin_local_position[0]:.2f}, {self.origin_local_position[1]:.2f}, {self.origin_local_position[2]:.2f}], alvo local=[{target_local[0]:.2f}, {target_local[1]:.2f}, {target_local[2]:.2f}], yaw_direção={self.direction_yaw:.1f}°")

    def land(self):
        """
        Inicia pouso via Offboard: vai até a posição home e desce até o chão.
        O pouso é controlado via TrajectorySetpoints.
        """
        self.get_logger().info("Iniciando POUSO via Offboard (ir para home e pousar)...")
        
        # Configura origem da trajetória
        self.origin_local_position = [
            self.PX4_local_position.x,
            self.PX4_local_position.y,
            self.PX4_local_position.z
        ]
        
        # Alvo: posição home (0, 0) no frame local, Z = 0 (chão)
        self.target_local_position = [0.0, 0.0, 0.0]
        
        # Calcula yaw de direção para apontar para home
        dx = 0.0 - self.PX4_local_position.x
        dy = 0.0 - self.PX4_local_position.y
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            self.direction_yaw = math.degrees(math.atan2(dy, dx))
            self.direction_yaw = (self.direction_yaw + 360) % 360
        else:
            # Já está no home, mantém yaw atual
            self.direction_yaw = self.PX4_current_yaw_deg_normalized
        
        # Sem yaw alvo final específico
        self.target_yaw = None
        
        # Não desarma automaticamente após pouso (diferença do RTL)
        self.rtl_auto_disarm = False
        
        # Marca início da trajetória
        self.on_trajectory = True
        self.trajectory_start_time = time.time()
        
        # Transiciona para estado de pouso
        self.state_machine._transition_to(DroneStateDescription.VOANDO_POUSANDO)
        
        self.get_logger().info(
            f"LAND configurado: origem=[{self.origin_local_position[0]:.2f}, "
            f"{self.origin_local_position[1]:.2f}, {-self.origin_local_position[2]:.2f}m], "
            f"alvo=home [0, 0, 0], yaw_direção={self.direction_yaw:.1f}°"
        )

    def return_to_launch(self):
        """
        Inicia retorno à base via Offboard com sequência:
        1. Gira para apontar na direção do home
        2. Sobe até a altitude RTL (30m) enquanto vai para o home
        3. Ao chegar na posição home (X=0, Y=0), transiciona para VOANDO_POUSANDO
        4. Pousa e desarma automaticamente
        """
        self.get_logger().info(
            f"Iniciando RTL via Offboard: girar → subir {self.rtl_altitude}m → ir para home → pousar → desarmar..."
        )
        
        # Configura origem da trajetória
        self.origin_local_position = [
            self.PX4_local_position.x,
            self.PX4_local_position.y,
            self.PX4_local_position.z
        ]
        
        # Alvo: posição home (0, 0) na altitude RTL (Z negativo = para cima no frame NED)
        self.target_local_position = [0.0, 0.0, -self.rtl_altitude]
        
        # Calcula yaw de direção para apontar para home
        dx = 0.0 - self.PX4_local_position.x
        dy = 0.0 - self.PX4_local_position.y
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            self.direction_yaw = math.degrees(math.atan2(dy, dx))
            self.direction_yaw = (self.direction_yaw + 360) % 360
        else:
            # Já está no home, mantém yaw atual
            self.direction_yaw = self.PX4_current_yaw_deg_normalized
        
        # Sem yaw alvo final específico
        self.target_yaw = None
        
        # Flags do RTL: após chegar ao destino, transiciona para POUSANDO e depois desarma
        self.rtl_pending_land = True
        self.rtl_auto_disarm = True
        
        # Marca início da trajetória
        self.on_trajectory = True
        self.trajectory_start_time = time.time()
        
        # Inicia na fase de rotação para direção do home
        self.state_machine._transition_to(DroneStateDescription.VOANDO_GIRANDO_INICIO)
        
        self.get_logger().info(
            f"RTL configurado: origem=[{self.origin_local_position[0]:.2f}, "
            f"{self.origin_local_position[1]:.2f}, {-self.origin_local_position[2]:.2f}m], "
            f"alvo=home [0, 0, {self.rtl_altitude}m], yaw_direção={self.direction_yaw:.1f}°"
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
        dx = self.PX4_local_position.x - self.target_local_position[0]
        dy = self.PX4_local_position.y - self.target_local_position[1]
        dz = self.PX4_local_position.z - self.target_local_position[2]
        
        distance_2d = math.sqrt(dx**2 + dy**2)
        distance_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Verifica se está na posição alvo
        position_reached = distance_3d <= self.position_tolerance
        
        # Verifica se o yaw está no alvo (se especificado)
        yaw_reached = True
        yaw_diff = None
        if self.target_yaw is not None:
            yaw_diff = self.PX4_current_yaw_deg_normalized - self.target_yaw
            # Normaliza para lidar com transição 360°->0°
            if yaw_diff < -180:
                yaw_diff += 360
            if yaw_diff > 180:
                yaw_diff -= 360
            yaw_reached = abs(yaw_diff) <= self.yaw_tolerance_deg
        
        self.get_logger().info(
            f"Verificação de chegada: distância={distance_3d:.2f}m (tol: {self.position_tolerance}m), "
            f"yaw_diff={abs(yaw_diff) if yaw_diff is not None else 'N/A'}° (tol: {self.yaw_tolerance_deg}°)",
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
    POUSADO_ARMANDO = 1       # Processo de arm em andamento no chão
    POUSADO_ARMADO = 2        # No chão, armado
    POUSADO_DESARMANDO = 3    # Processo de desarm em andamento no chão
    
    # Estados de voo estável (10-19)
    VOANDO_PRONTO = 10        # Voando estável, aguardando comando (hover)
    
    # Estados de trajetória/movimento (20-29)
    VOANDO_DECOLANDO = 20     # Subindo até altitude de decolagem
    VOANDO_GIRANDO_INICIO = 21  # Girando para direção do target
    VOANDO_A_CAMINHO = 22     # Voando em direção ao target
    VOANDO_GIRANDO_FIM = 23   # Girando para yaw final
    VOANDO_POUSANDO = 24      # Descendo para pousar
    VOANDO_RTL = 25           # Retornando para home
    
    # Emergência (99)
    EMERGENCIA = 99           # Failsafe/emergência ativa


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
        "DISARM": [DroneStateDescription.POUSADO_ARMADO, DroneStateDescription.VOANDO_PRONTO,
                   DroneStateDescription.VOANDO_DECOLANDO, DroneStateDescription.VOANDO_GIRANDO_INICIO,
                   DroneStateDescription.VOANDO_A_CAMINHO, DroneStateDescription.VOANDO_GIRANDO_FIM],
        "TAKEOFF": [DroneStateDescription.POUSADO_ARMADO],
        "GOTO": [DroneStateDescription.VOANDO_PRONTO],
        "LAND": [DroneStateDescription.VOANDO_PRONTO, DroneStateDescription.VOANDO_DECOLANDO,
                 DroneStateDescription.VOANDO_GIRANDO_INICIO, DroneStateDescription.VOANDO_A_CAMINHO,
                 DroneStateDescription.VOANDO_GIRANDO_FIM],
        "RTL": [DroneStateDescription.VOANDO_PRONTO, DroneStateDescription.VOANDO_DECOLANDO,
                DroneStateDescription.VOANDO_GIRANDO_INICIO, DroneStateDescription.VOANDO_A_CAMINHO,
                DroneStateDescription.VOANDO_GIRANDO_FIM],
    }
    
    def __init__(self, node: 'DroneNode'):
        """
        Inicializa a máquina de estados.
        
        Args:
            node: Referência ao DroneNode para acessar variáveis de estado do drone
        """
        self.node = node
        self.state = DroneStateDescription.POUSADO_DESARMADO
        self.state_entry_time = time.time()
        
        # Histórico de transições para debug
        self.state_history = []
        self.max_history_size = 50
    
    def _check_transitions(self):
        """
        Verifica e executa transições automáticas baseadas nas condições do drone.
        Usa o enum unificado DroneStateDescription.
        """
        n = self.node  # Atalho para o nó
        
        # === PRIORIDADE 1: Verificar emergência ===
        if self._check_emergency_conditions():
            self._transition_to(DroneStateDescription.EMERGENCIA)
            return
        
        # === PRIORIDADE 2: Estados de armamento/desarmamento em solo ===
        
        # Se desarmando (comando enviado) e ainda reportado armado
        if n.is_disarming and n.PX4_is_armed:
            self._transition_to(DroneStateDescription.POUSADO_DESARMANDO)
            return
        
        # Se desarmado e não está armando
        if not n.PX4_is_armed and not n.is_arming:
            self._transition_to(DroneStateDescription.POUSADO_DESARMADO)
            n.is_disarming = False
            return
        
        # Se está armando
        if n.is_arming and not n.PX4_is_armed:
            self._transition_to(DroneStateDescription.POUSADO_ARMANDO)
            return
        
        # === PRIORIDADE 3: Estados de voo (armado) ===
        if n.PX4_is_armed:
            n.is_disarming = False
            
            # Armado mas no chão e sem trajetória ativa
            if n.PX4_is_landed and not n.on_trajectory:
                self._transition_to(DroneStateDescription.POUSADO_ARMADO)
                return

            # Em trajetória Offboard (fases) - transições centralizadas aqui
            if self.state in (
                DroneStateDescription.VOANDO_DECOLANDO,
                DroneStateDescription.VOANDO_GIRANDO_INICIO,
                DroneStateDescription.VOANDO_A_CAMINHO,
                DroneStateDescription.VOANDO_GIRANDO_FIM,
                DroneStateDescription.VOANDO_POUSANDO,
            ):

                # Proteções: precisa de posição local e alvo para avaliar transições
                if n.PX4_local_position is None or n.target_local_position is None:
                    return

                current_x = n.PX4_local_position.x
                current_y = n.PX4_local_position.y
                current_z = n.PX4_local_position.z

                target_x, target_y, target_z = n.target_local_position

                dx = target_x - current_x
                dy = target_y - current_y
                dz = target_z - current_z
                distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)

                # VOANDO_DECOLANDO -> VOANDO_PRONTO
                if self.state == DroneStateDescription.VOANDO_DECOLANDO:
                    current_alt = -current_z
                    target_alt = -target_z
                    if abs(current_alt - target_alt) <= n.position_tolerance:
                        n.stop_movement()
                        self._transition_to(DroneStateDescription.VOANDO_PRONTO)
                        self.node.get_logger().info(
                            f"Decolagem completa! Altitude atual: {current_alt:.2f}m"
                        )
                    return

                # VOANDO_GIRANDO_INICIO -> VOANDO_A_CAMINHO
                if self.state == DroneStateDescription.VOANDO_GIRANDO_INICIO:
                    if n.PX4_is_landed:
                        self._transition_to(DroneStateDescription.VOANDO_A_CAMINHO)
                        self.node.get_logger().info(
                            "Drone está pousado. Pulando rotação inicial e iniciando movimento direto."
                        )
                        return

                    if n.direction_yaw is None:
                        return

                    yaw_diff = n.PX4_current_yaw_deg_normalized - n.direction_yaw
                    if yaw_diff < -180:
                        yaw_diff += 360
                    if yaw_diff > 180:
                        yaw_diff -= 360

                    if abs(yaw_diff) <= n.yaw_tolerance_deg:
                        self._transition_to(DroneStateDescription.VOANDO_A_CAMINHO)
                        self.node.get_logger().info(
                            f"Yaw de direção alcançado ({n.direction_yaw:.1f}°). Iniciando movimento."
                        )
                    return

                # VOANDO_A_CAMINHO -> VOANDO_GIRANDO_FIM | VOANDO_PRONTO | VOANDO_POUSANDO (RTL)
                if self.state == DroneStateDescription.VOANDO_A_CAMINHO:
                    if distance_to_target <= n.position_tolerance:
                        # RTL: ao chegar no home (altitude RTL), transiciona para pouso
                        if n.rtl_pending_land:
                            n.rtl_pending_land = False
                            # Atualiza alvo para descer até o chão (home no Z=0)
                            n.target_local_position = [0.0, 0.0, 0.0]
                            self._transition_to(DroneStateDescription.VOANDO_POUSANDO)
                            self.node.get_logger().info(
                                "RTL: Posição home alcançada. Iniciando descida para pouso..."
                            )
                        elif n.target_yaw is not None:
                            self._transition_to(DroneStateDescription.VOANDO_GIRANDO_FIM)
                            self.node.get_logger().info(
                                f"Posição alvo alcançada. Iniciando rotação para yaw alvo ({n.target_yaw:.1f}°)."
                            )
                        else:
                            n.stop_movement()
                            self._transition_to(DroneStateDescription.VOANDO_PRONTO)
                            self.node.get_logger().info("Posição alvo alcançada. Trajetória completa.")
                    return

                # VOANDO_GIRANDO_FIM -> VOANDO_PRONTO
                if self.state == DroneStateDescription.VOANDO_GIRANDO_FIM:
                    if n.target_yaw is None:
                        n.stop_movement()
                        self._transition_to(DroneStateDescription.VOANDO_PRONTO)
                        return

                    yaw_diff = n.PX4_current_yaw_deg_normalized - n.target_yaw
                    if yaw_diff < -180:
                        yaw_diff += 360
                    if yaw_diff > 180:
                        yaw_diff -= 360

                    if abs(yaw_diff) <= n.yaw_tolerance_deg:
                        n.stop_movement()
                        self._transition_to(DroneStateDescription.VOANDO_PRONTO)
                        self.node.get_logger().info(
                            f"Yaw alvo alcançado ({n.target_yaw:.1f}°). Trajetória completa."
                        )
                    return

                # VOANDO_POUSANDO -> POUSADO_ARMADO | POUSADO_DESARMANDO (quando pousou)
                if self.state == DroneStateDescription.VOANDO_POUSANDO:
                    if n.PX4_is_landed:
                        n.stop_movement()
                        if n.rtl_auto_disarm:
                            n.rtl_auto_disarm = False
                            n.disarm()
                            self._transition_to(DroneStateDescription.POUSADO_DESARMANDO)
                            self.node.get_logger().info("Pouso completo. Desarmando automaticamente (RTL)...")
                        else:
                            self._transition_to(DroneStateDescription.POUSADO_ARMADO)
                            self.node.get_logger().info("Pouso completo. Drone armado no chão.")
                    return

                return
            
            # Voando mas sem trajetória = VOANDO_PRONTO (hover)
            if not n.PX4_is_landed and not n.on_trajectory:
                # Não sobrescreve o estado POUSANDO que ainda está em processo de descida
                if self.state != DroneStateDescription.VOANDO_POUSANDO:
                    self._transition_to(DroneStateDescription.VOANDO_PRONTO)
                    return
    
    def _update_trajectory_state(self):
        """
        Mantido por compatibilidade, mas a fase agora é controlada diretamente
        pelo `state` (DroneStateDescription). Não faz mais mapeamento por string.
        """
        return
    
    def _transition_to(self, new_state: DroneStateDescription):
        """
        Executa transição para novo estado.
        
        Args:
            new_state: Novo estado para transicionar
        """
        if self.state != new_state:
            self.state = new_state
            self.state_entry_time = time.time()
    
    def _on_state_changed(self, old_state: DroneStateDescription):
        """
        Callback executado quando o estado muda.
        Registra no histórico e loga a transição.
        
        Args:
            old_state: Estado anterior
        """
        # Registra no histórico
        self.state_history.append({
            'time': time.time(),
            'from_state': old_state.name,
            'to_state': self.state.name
        })
        
        # Limita tamanho do histórico
        if len(self.state_history) > self.max_history_size:
            self.state_history.pop(0)
        
        # Log da transição
        self.node.get_logger().info(
            f"Estado: {old_state.name} -> {self.state.name}"
        )
    
    def _check_emergency_conditions(self) -> bool:
        """
        Verifica condições de emergência que requerem transição imediata.
        
        Returns:
            bool: True se há condição de emergência ativa
        """
        n = self.node
        
        # Bateria crítica (menos de 10%)
        if n.PX4_battery_status and n.PX4_battery_status.remaining < 0.10:
            self.node.get_logger().warn("EMERGÊNCIA: Bateria crítica!")
            return True
        
        # Adicione outras condições de emergência aqui conforme necessário
        # Exemplo: perda de GPS, failsafe do PX4, etc.
        
        return False
    
    def can_execute_command(self, command: str) -> tuple:
        """
        Verifica se um comando pode ser executado no estado atual.
        
        Args:
            command: Nome do comando (ARM, DISARM, TAKEOFF, GOTO, LAND, RTL)
        
        Returns:
            tuple: (pode_executar: bool, mensagem_erro: str)
        """
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
    
    def get_status_dict(self) -> dict:
        """
        Retorna dicionário com informações de estado para publicação de status.
        
        Returns:
            dict: Dicionário com estado e duração (sem sub-estado)
        """
        return {
            "drone_state": self.state.name,
            "state_duration_sec": round(time.time() - self.state_entry_time, 2),
        }
    
    def get_state(self) -> DroneStateDescription:
        """Retorna o estado atual."""
        return self.state





def main(args=None):
    """Função principal do nó."""
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    drone_node = DroneNode()

    try:
        rclpy.spin(drone_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        drone_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()




