# fsm_node.py
# =================================================================================================
# NÓ DA MÁQUINA DE ESTADOS FINITA (O "CÉREBRO")
# =================================================================================================
# RESPONSABILIDADE PRINCIPAL:
# Gerenciar o fluxo lógico e sequencial da missão de inspeção. Implementa uma FSM hierárquica
# com estados e sub-estados para controlar o comportamento do drone durante
# toda a missão.
#
# =================================================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import time
import math # Para cálculos de ângulos e distâncias
from enum import IntEnum

# Importações das mensagens ROS customizadas
from drone_inspetor_msgs.msg import (
    DroneStateMSG,
    FSMStateMSG,
    CVDetectionMSG,
    CVDetectionItemMSG,
    MissionCommandMSG,
)


# ==================================================================================================
# ENUM DE ESTADOS DA FSM - UNIFICADO
# ==================================================================================================

class FSMStateDescription(IntEnum):
    """
    Enum unificado que representa todos os estados possíveis da FSM.
    Substitui os 4 enums anteriores (FSMState, FSMSubState, FSMInspectionState, FSMScanState).
    """
    # Estados de nível 0 - Sistema
    DESATIVADO = 0                              # Sistema desativado, aguardando tópicos essenciais
    PRONTO = 1                                  # Pronto para iniciar missão
    
    # Estados EXECUTANDO_MISSAO - Nível 1
    EXECUTANDO_ARMANDO = 10                     # Armando motores
    EXECUTANDO_DECOLANDO = 11                   # Decolagem em progresso
    EXECUTANDO_VOANDO = 12                      # Voando para ponto de inspeção
    
    # Estados INSPECIONANDO - Nível 2
    EXECUTANDO_INSPECIONANDO_DETECTANDO = 20    # Girando 360° procurando target
    EXECUTANDO_INSPECIONANDO_CENTRALIZANDO = 21 # Centralizando target no frame
    EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA = 22  # Escaneando sem anomalia detectada
    EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA = 23     # Anomalia detectada durante escaneamento
    EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO = 24       # Focando na anomalia detectada
    EXECUTANDO_INSPECIONANDO_FALHA = 25         # Falha na detecção do target
    
    # Estados de retorno
    INSPECAO_FINALIZADA = 30                    # Pausa pós-inspeção
    RETORNO_RETORNANDO = 40                     # RTL em progresso
    RETORNO_DESARMANDO = 41                     # Desarmando após pouso


# ==================================================================================================
# CLASSE DroneStateData
# ==================================================================================================

class DroneStateData:
    """
    Encapsula todos os dados recebidos do drone_node.
    Representa o estado atual do drone conforme reportado pelo drone_node.
    """
    
    def __init__(self):
        """Inicializa todas as variáveis do drone com valores padrão."""
        # Status de controle
        self.offboard_mode = False
        self.armed = False
        self.landed = False
        self.on_trajectory = False
        
        # Posição local (NED - North East Down)
        self.local_position_x = 0.0
        self.local_position_y = 0.0
        self.local_position_z = 0.0
        
        # Posição global (GPS)
        self.global_position_lat = 0.0
        self.global_position_lon = 0.0
        self.global_position_alt = 0.0
        
        # Orientação
        self.yaw = 0.0
        
        # Estado da máquina de estados do drone (unificado, sem sub_state)
        self.state = "POUSADO_DESARMADO"
        self.state_duration = 0.0
    
    def reset(self):
        """Reseta variáveis para valores padrão."""
        self.__init__()
    
    def update_from_msg(self, msg: DroneStateMSG):
        """
        Atualiza valores a partir de mensagem ROS DroneStateMSG.
        
        Args:
            msg: Mensagem DroneStateMSG recebida do drone_node
        """
        self.offboard_mode = msg.offboard_mode_active
        self.armed = msg.is_armed
        self.landed = msg.is_landed
        self.on_trajectory = msg.on_trajectory
        
        self.local_position_x = msg.local_x
        self.local_position_y = msg.local_y
        self.local_position_z = msg.local_z
        
        self.global_position_lat = msg.latitude
        self.global_position_lon = msg.longitude
        self.global_position_alt = msg.altitude
        
        self.yaw = msg.yaw_deg
        
        self.state = msg.state_name
        self.state_duration = msg.state_duration_sec


# ==================================================================================================
# CLASSE FSMStateData
# ==================================================================================================

class FSMStateData:
    """
    Encapsula todas as variáveis de estado da FSM.
    Contém o estado atual e todas as variáveis de controle da missão.
    """
    
    def __init__(self, node: 'FSMNode'):
        """
        Inicializa todas as variáveis de estado da FSM.
        
        Args:
            node: Referência ao FSMNode para acesso ao publisher
        """
        self._node = node
        
        # Estado atual e anterior (inteiros)
        self.state = FSMStateDescription.DESATIVADO
        self.state_anterior = FSMStateDescription.DESATIVADO
        
        # Controle de aguardo de comando
        self.waiting_for_state = None
        self.command_timeout = 10.0
        self.command_sent_time = 0.0
        
        # Parâmetros da missão
        self.mission_started = False
        self.inspection_type = "Flare"
        self.takeoff_altitude = 20.0
        self.inspection_distance = 5.0
        self.anomaly_approach_distance = 2.0
        self.waypoint_tolerance = 0.5
        
        # Pontos de inspeção
        self.inspection_point_lat = -22.634010
        self.inspection_point_lon = -40.092463
        self.inspection_point_alt = 99.0
        
        # Target para órbita
        self.target_lat = -22.633061
        self.target_lon = -40.093330
        self.target_alt = 99.0
        
        # Parâmetros do escaneamento
        self.current_scanning_angle = 0.0
        self.anomaly_angles_list = []
        
        # Estados de controle interno
        self.target_detected = False
        self.anomaly_detected = False
        
        # Variáveis de detecção de flare
        self.flare_detected = False
        self.flare_detection_start_yaw = None
        self.flare_detection_yaw_covered = 0.0
        self.flare_detection_current_target_yaw = None
        self.flare_detection_increment = 4.5
        self.flare_detection_yaw_reached = False
        
        # Variáveis de centralização do target
        self.target_centralized = False
        self.target_bbox_center = None
        self.target_yaw_offset = 0.0
        self.centralization_tolerance = 2.0
        
        # Variáveis de estabilidade e inspeção
        self.is_stable = False
        self.stability_check_start_time = 0.0
        self.inspection_completed = False
        self.inspection_start_angle = 0.0
        self.current_inspection_angle = 0.0
        self.last_anomaly_angle = 0.0
        self.failure_pause_start_time = 0.0
        self.inspection_pause_start_time = 0.0
        self.anomaly_focus_start_time = 0.0

        # Controle de habilitação de envio OffboardControlMode/TrajectorySetpoint no drone_node
        self.OffboardControlMode_enabled = False
    
    def reset(self):
        """
        Reseta variáveis para nova missão.
        Mantém valores de configuração (altitudes, tolerâncias, etc.).
        """
        self.state = FSMStateDescription.DESATIVADO
        self.state_anterior = FSMStateDescription.DESATIVADO
        
        self.waiting_for_state = None
        self.command_sent_time = 0.0
        self.command_timeout = 10.0
        
        self.mission_started = False
        self.inspection_type = "Flare"
        
        self.current_scanning_angle = 0.0
        self.anomaly_angles_list = []
        
        self.target_detected = False
        self.anomaly_detected = False
        
        self.flare_detected = False
        self.flare_detection_start_yaw = None
        self.flare_detection_yaw_covered = 0.0
        self.flare_detection_current_target_yaw = None
        self.flare_detection_yaw_reached = False
        
        self.target_centralized = False
        self.target_bbox_center = None
        self.target_yaw_offset = 0.0
        
        self.is_stable = False
        self.stability_check_start_time = 0.0
        self.inspection_completed = False
        self.inspection_start_angle = 0.0
        self.current_inspection_angle = 0.0
        self.last_anomaly_angle = 0.0
        self.failure_pause_start_time = 0.0
        self.inspection_pause_start_time = 0.0
        self.anomaly_focus_start_time = 0.0
    
    def publish(self):
        """
        Publica estado atual no tópico ROS.
        Cria mensagem FSMState e publica através do publisher do node.
        """
        msg = FSMStateMSG()
        msg.state = int(self.state)
        msg.state_name = self.state.name
        
        msg.waiting_for_state = self.waiting_for_state if self.waiting_for_state else ""
        msg.command_timeout = self.command_timeout
        msg.command_sent_time = self.command_sent_time
        
        msg.mission_started = self.mission_started
        msg.inspection_type = self.inspection_type
        msg.takeoff_altitude = self.takeoff_altitude
        msg.inspection_distance = self.inspection_distance
        msg.anomaly_approach_distance = self.anomaly_approach_distance
        msg.waypoint_tolerance = self.waypoint_tolerance
        
        msg.inspection_point_lat = self.inspection_point_lat
        msg.inspection_point_lon = self.inspection_point_lon
        msg.inspection_point_alt = self.inspection_point_alt
        
        msg.target_lat = self.target_lat
        msg.target_lon = self.target_lon
        msg.target_alt = self.target_alt
        
        msg.current_scanning_angle = self.current_scanning_angle
        
        msg.target_detected = self.target_detected
        msg.anomaly_detected = self.anomaly_detected
        
        msg.flare_detected = self.flare_detected
        msg.flare_detection_start_yaw = self.flare_detection_start_yaw if self.flare_detection_start_yaw is not None else 0.0
        msg.flare_detection_yaw_covered = self.flare_detection_yaw_covered
        msg.flare_detection_current_target_yaw = self.flare_detection_current_target_yaw if self.flare_detection_current_target_yaw is not None else 0.0
        msg.flare_detection_increment = self.flare_detection_increment
        msg.flare_detection_yaw_reached = self.flare_detection_yaw_reached
        
        msg.target_centralized = self.target_centralized
        msg.target_yaw_offset = self.target_yaw_offset
        msg.centralization_tolerance = self.centralization_tolerance
        
        msg.is_stable = self.is_stable
        msg.stability_check_start_time = self.stability_check_start_time
        msg.inspection_completed = self.inspection_completed
        msg.inspection_start_angle = self.inspection_start_angle
        msg.current_inspection_angle = self.current_inspection_angle
        msg.last_anomaly_angle = self.last_anomaly_angle
        msg.failure_pause_start_time = self.failure_pause_start_time
        msg.inspection_pause_start_time = self.inspection_pause_start_time
        
        self._node.fsm_state_pub.publish(msg)


# ==================================================================================================
# CLASSE FSMNode
# ==================================================================================================

class FSMNode(Node):
    """
    Implementa a lógica de estados hierárquica da missão de inspeção.
    
    FLUXO DA MISSÃO:
    1. DESATIVADO: Falha em algum dos tópicos essenciais
    2. PRONTO: Drone pronto e aguardando comando "start_inspection" do dashboard
    3. EXECUTANDO_ARMANDO: Arma os motores
    4. EXECUTANDO_DECOLANDO: Decola para altitude de missão
    5. EXECUTANDO_VOANDO: Navega para Ponto de Detecção
    6. EXECUTANDO_INSPECIONANDO_DETECTANDO: Gira 360° procurando target
    7. EXECUTANDO_INSPECIONANDO_CENTRALIZANDO: Centraliza target no frame
    8. EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA: Voo orbital ao redor do target
    9. EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA: Detecção de anomalia durante escaneamento
    10. EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO: Aproxima e foca na anomalia por 5s
    11. INSPECAO_FINALIZADA: Pausa por 3s
    12. RETORNO_RETORNANDO: RTL e pousa
    13. RETORNO_DESARMANDO: Desarma e finaliza
    """
    
    def __init__(self):
        # --- Inicialização do Nó ROS2 ---
        super().__init__("fsm_node")
        self.get_logger().info("================ INICIALIZANDO FSM NODE ===============")

        # QoS para comandos críticos: TRANSIENT_LOCAL + BEST_EFFORT
        qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS para status do sistema: TRANSIENT_LOCAL + BEST_EFFORT
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS para dados de sensores: VOLATILE + BEST_EFFORT
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- Instâncias das Classes de Estado ---
        self.drone = DroneStateData()
        self.fsm_state = FSMStateData(self)
        
        # --- Verificação de Saúde dos Tópicos Essenciais ---
        self.essencial_topics = {
            "drone_state": {"last_received": 0.0, "description": "Estado do drone_node"},
        }
        self.topic_health_timeout = 2.0

        # ==================================================================
        # PUBLISHERS
        # ==================================================================
        # Publica o estado atual da FSM para o dashboard
        self.fsm_state_pub = self.create_publisher(FSMStateMSG, "/drone_inspetor/interno/fsm_node/fsm_state", qos_status)
        
        # Envia comandos para o drone_node (usando MissionCommandMSG)
        self.fsm_command_pub = self.create_publisher(MissionCommandMSG, "/drone_inspetor/interno/fsm_node/drone_commands", qos_commands)

        # ==================================================================
        # SUBSCRIBERS
        # ==================================================================
        # Recebe estado do drone_node
        self.drone_state_sub = self.create_subscription(
            DroneStateMSG, "/drone_inspetor/interno/drone_node/drone_state", self.drone_state_callback, qos_status)
        
        # Recebe comandos do dashboard_node
        self.dashboard_command_sub = self.create_subscription(
            String, "/drone_inspetor/interno/dashboard_node/mission_commands", self.dashboard_command_callback, qos_commands)
        
        # Recebe detecções do cv_node
        self.cv_detection_sub = self.create_subscription(
            CVDetectionMSG, "/drone_inspetor/interno/cv_node/object_detections", self.cv_detection_callback, qos_sensor_data)

        # ==================================================================
        # TIMERS
        # ==================================================================
        # Timer principal da FSM - executa a lógica de estados a cada 100ms
        self.fsm_timer = self.create_timer(0.1, self.fsm_step)
        
        # Timer para publicação periódica do estado FSM
        self.fsm_state_pub_timer = self.create_timer(0.5, lambda: self.fsm_state.publish())
        
        # Timer para verificação de saúde dos tópicos essenciais - executa a cada 2 segundos
        self.topic_health_timer = self.create_timer(2.0, self.check_essential_topics)
        
        self.get_logger().info("================ FSM NODE PRONTO ================")

    # ==================================================================
    # LÓGICA PRINCIPAL DA FSM
    # ==================================================================

    def fsm_step(self):
        """
        O coração da FSM, executado a cada 100ms.
        Implementa a máquina de estados completa usando match/case.
        """
        current_state = self.fsm_state.state

        # Verificação especial para estado DESATIVADO (antes do match)
        if current_state == FSMStateDescription.DESATIVADO:
            if not self.drone.offboard_mode:
                self.get_logger().info(f">>> Aguardando mudança para Modo OFFBOARD!", throttle_duration_sec=2)
                return

            self.get_logger().info("Sistema no modo OFFBOARD. Transicionando para Estado PRONTO!")
            self.set_state(FSMStateDescription.PRONTO)
            return
        
        # Verifica se o drone saiu do modo OFFBOARD
        if not self.drone.offboard_mode:
            self.get_logger().error(f"Drone saiu do modo OFFBOARD. Resetando FSM...")
            self.reset_FSM()
            return

        # =================================================
        # MÁQUINA DE ESTADOS USANDO MATCH/CASE
        # =================================================
        match current_state:
            # =================================================
            # ESTADO: PRONTO
            # =================================================
            case FSMStateDescription.PRONTO:
                if self.fsm_state.mission_started:
                    self.get_logger().info("\033[1m\033[31mINICIANDO MISSÃO: Iniciando sequência de missão!\033[0m")
                    self.fsm_state.mission_started = False
                    self.get_logger().info(f"INICIANDO MISSÃO: Tipo de inspeção selecionada: {self.fsm_state.inspection_type}")
                    self.get_logger().info("INICIANDO MISSÃO: Transicionando para estado EXECUTANDO_ARMANDO...")
                    self.set_state(FSMStateDescription.EXECUTANDO_ARMANDO)
                    return
                
                self.get_logger().info("Estado PRONTO: Comando para iniciar a missão não recebido. Aguardando comando...", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_ARMANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_ARMANDO:
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Aguardando drone armar... Estado atual: {self.drone.state}", throttle_duration_sec=2)
                    return
                elif wait_status == "timeout":
                    self.get_logger().error("Timeout ao armar! Abortando missão.")
                    self.set_state(FSMStateDescription.DESATIVADO)
                    return

                if self.drone.state == "POUSADO_ARMADO":
                    self.get_logger().info("Drone ARMADO confirmado. Transicionando para DECOLANDO...")
                    self.set_state(FSMStateDescription.EXECUTANDO_DECOLANDO)
                    return

                if self.drone.state == "POUSADO_DESARMADO":
                    self.get_logger().info("Enviando comando ARM e aguardando confirmação...")
                    self.send_command_and_wait({"command": "ARM"}, "POUSADO_ARMADO", timeout=10.0)
                    return
                
                self.get_logger().warn(f"Estado inesperado do drone durante ARMANDO: {self.drone.state}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_DECOLANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_DECOLANDO:
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Aguardando decolagem... Estado: {self.drone.state}", throttle_duration_sec=2)
                    return
                elif wait_status == "timeout":
                    self.get_logger().error("Timeout na decolagem! Abortando missão.")
                    self.set_state(FSMStateDescription.DESATIVADO)
                    return

                if self.drone.state == "VOANDO_PRONTO":
                    self.get_logger().info("Decolagem concluída! Drone em VOANDO_PRONTO. Transicionando para VOANDO...")
                    self.set_state(FSMStateDescription.EXECUTANDO_VOANDO)
                    return

                if self.drone.state == "POUSADO_ARMADO":
                    self.get_logger().info(f"Enviando comando TAKEOFF para {self.fsm_state.takeoff_altitude}m...")
                    self.send_command_and_wait(
                        {"command": "TAKEOFF", "alt": self.fsm_state.takeoff_altitude},
                        "VOANDO_PRONTO",
                        timeout=30.0
                    )
                    return
                
                if self.drone.state == "VOANDO_DECOLANDO":
                    self.get_logger().info("Drone decolando...", throttle_duration_sec=2)
                    return
                
                self.get_logger().warn(f"Estado inesperado durante DECOLANDO: {self.drone.state}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_VOANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_VOANDO:
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Voando para ponto de inspeção... Estado: {self.drone.state}", throttle_duration_sec=2)
                    return

                if self.drone.state == "VOANDO_PRONTO":
                    drone_reached_waypoint = self.is_at_waypoint(
                        self.fsm_state.inspection_point_lat,
                        self.fsm_state.inspection_point_lon,
                        self.fsm_state.inspection_point_alt
                    )

                    if drone_reached_waypoint:
                        self.get_logger().info("Drone chegou ao ponto de inspeção. Transicionando para INSPECIONANDO_DETECTANDO...")
                        self.set_state(FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO)
                        return

                    self.get_logger().info(f"Navegando para ponto de inspeção ({self.fsm_state.inspection_point_lat:.6f}, {self.fsm_state.inspection_point_lon:.6f}, {self.fsm_state.inspection_point_alt:.1f})...")
                    self.send_command_and_wait(
                        {"command": "GOTO", "lat": self.fsm_state.inspection_point_lat, "lon": self.fsm_state.inspection_point_lon, "alt": self.fsm_state.inspection_point_alt},
                        "VOANDO_PRONTO",
                        timeout=60.0
                    )
                    return
                
                # Estados de trajetória (novos estados unificados)
                if self.drone.state in ["VOANDO_GIRANDO_INICIO", "VOANDO_A_CAMINHO", "VOANDO_GIRANDO_FIM"]:
                    self.get_logger().info(f"Em trajetória... Estado: {self.drone.state}", throttle_duration_sec=2)
                    return
                
                self.get_logger().warn(f"Estado inesperado durante VOANDO: {self.drone.state}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_DETECTANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO:
                if not self.fsm_state.flare_detected:
                    if self.fsm_state.flare_detection_start_yaw is None:
                        self.fsm_state.flare_detection_start_yaw = self.drone.yaw
                        self.fsm_state.flare_detection_current_target_yaw = self.drone.yaw
                        self.fsm_state.flare_detection_yaw_covered = 0.0
                        self.fsm_state.flare_detection_yaw_reached = False
                        self.get_logger().info(f"Iniciando busca incremental de flare a partir do yaw {self.fsm_state.flare_detection_start_yaw:.1f}°...")
                    
                    if self.fsm_state.flare_detection_current_target_yaw is not None:
                        yaw_diff_to_target = self.drone.yaw - self.fsm_state.flare_detection_current_target_yaw
                        if yaw_diff_to_target < -180:
                            yaw_diff_to_target += 360
                        if yaw_diff_to_target > 180:
                            yaw_diff_to_target -= 360
                        
                        if abs(yaw_diff_to_target) <= 1.0:
                            if not self.fsm_state.flare_detection_yaw_reached:
                                self.fsm_state.flare_detection_yaw_reached = True
                                self.get_logger().info(f"Yaw alvo {self.fsm_state.flare_detection_current_target_yaw:.1f}° alcançado. Aguardando detecção...")
                        else:
                            self.get_logger().debug(f"Aguardando alcançar yaw {self.fsm_state.flare_detection_current_target_yaw:.1f}° (atual: {self.drone.yaw:.1f}°)", 
                                                   throttle_duration_sec=1)
                            return
                    
                    if self.fsm_state.flare_detection_yaw_reached:
                        yaw_diff = self.drone.yaw - self.fsm_state.flare_detection_start_yaw
                        if yaw_diff < -180:
                            yaw_diff += 360
                        if yaw_diff > 180:
                            yaw_diff -= 360
                        
                        self.fsm_state.flare_detection_yaw_covered = abs(yaw_diff)
                        
                        if self.fsm_state.flare_detection_yaw_covered >= 360.0:
                            self.get_logger().warn("Flare não detectado após busca completa de 360°.")
                            self.set_state(FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA)
                            return
                        
                        next_target_yaw = (self.fsm_state.flare_detection_current_target_yaw + self.fsm_state.flare_detection_increment) % 360.0
                        self.fsm_state.flare_detection_current_target_yaw = next_target_yaw
                        self.fsm_state.flare_detection_yaw_reached = False
                        
                        self.get_logger().info(f"Girando para próximo yaw alvo: {next_target_yaw:.1f}° (coberto: {self.fsm_state.flare_detection_yaw_covered:.1f}°)")
                        self.send_yaw_command(next_target_yaw)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_CENTRALIZANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO:
                if not self.fsm_state.target_centralized:
                    if self.fsm_state.target_yaw_offset != 0.0:
                        target_yaw = (self.drone.yaw + self.fsm_state.target_yaw_offset) % 360.0
                        yaw_diff = self.drone.yaw - target_yaw
                        if yaw_diff < -180:
                            yaw_diff += 360
                        if yaw_diff > 180:
                            yaw_diff -= 360
                        
                        if abs(yaw_diff) <= self.fsm_state.centralization_tolerance:
                            self.fsm_state.target_centralized = True
                            self.get_logger().info("Target centralizado! Transicionando para próximo estado...")
                        else:
                            self.get_logger().info(f"Centralizando target... Offset: {self.fsm_state.target_yaw_offset:.1f}° (atual: {self.drone.yaw:.1f}°, alvo: {target_yaw:.1f}°)", 
                                                  throttle_duration_sec=0.5)
                            self.send_yaw_command(target_yaw)
                    else:
                        self.get_logger().warn("Aguardando cálculo do offset de centralização...")
                
                if self.fsm_state.target_centralized:
                    self.set_state(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA:
                if not self.fsm_state.inspection_completed:
                    if self.fsm_state.inspection_start_angle == 0.0:
                        self.fsm_state.inspection_start_angle = time.time()
                        self.get_logger().warn("Voo orbital não implementado. Simulando escaneamento por 5 segundos...")
                    
                    elapsed = time.time() - self.fsm_state.inspection_start_angle
                    if elapsed >= 5.0:
                        self.get_logger().info("Escaneamento simulado concluído. Finalizando inspeção.")
                        self.fsm_state.inspection_completed = True
                        self.set_state(FSMStateDescription.INSPECAO_FINALIZADA)
                    else:
                        self.get_logger().info(f"Escaneando... {5.0 - elapsed:.1f}s restantes", throttle_duration_sec=1)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_FALHA
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA:
                if self.fsm_state.failure_pause_start_time == 0.0:
                    self.fsm_state.failure_pause_start_time = time.time()
                    self.get_logger().error("Falha na detecção do flare. Pausando por 3 segundos antes de abortar.")
                
                if time.time() - self.fsm_state.failure_pause_start_time >= 3.0:
                    self.get_logger().info("Pausa de falha concluída. Abortando inspeção.")
                    self.set_state(FSMStateDescription.INSPECAO_FINALIZADA)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA:
                self.get_logger().info("Anomalia detectada durante escaneamento!")
                self.fsm_state.anomaly_focus_start_time = time.time()
                self.set_state(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO:
                elapsed_time = time.time() - self.fsm_state.anomaly_focus_start_time
                remaining_time = 5.0 - elapsed_time
                
                self.get_logger().info(f"Focando anomalia... {remaining_time:.1f}s restantes", 
                                    throttle_duration_sec=1)
                
                if elapsed_time >= 5.0:
                    self.get_logger().info("5 segundos de foco concluídos. Retornando ao escaneamento.")
                    self.fsm_state.anomaly_detected = False
                    self.set_state(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA)

            # =================================================
            # ESTADO: INSPECAO_FINALIZADA
            # =================================================
            case FSMStateDescription.INSPECAO_FINALIZADA:
                if self.fsm_state.inspection_pause_start_time == 0.0:
                    self.fsm_state.inspection_pause_start_time = time.time()
                    self.get_logger().info("Inspeção finalizada. Pausando por 3 segundos...")
                
                if time.time() - self.fsm_state.inspection_pause_start_time >= 3.0:
                    self.get_logger().info("Pausa concluída. Iniciando retorno ao helideck.")
                    self.set_state(FSMStateDescription.RETORNO_RETORNANDO)

            # =================================================
            # ESTADO: RETORNO_RETORNANDO
            # =================================================
            case FSMStateDescription.RETORNO_RETORNANDO:
                if not self.drone.state.startswith("POUSADO"):
                    self.get_logger().info("Enviando comando RTL...", throttle_duration_sec=2)
                    self.publish_drone_command({"command": "RTL"})
                else:
                    self.get_logger().info("Drone pousou. Transicionando para DESARMANDO...")
                    self.set_state(FSMStateDescription.RETORNO_DESARMANDO)

            # =================================================
            # ESTADO: RETORNO_DESARMANDO
            # =================================================
            case FSMStateDescription.RETORNO_DESARMANDO:
                if self.drone.state.startswith("POUSADO"):
                    self.get_logger().info("Enviando comando DISARM...")
                    self.publish_drone_command({"command": "DISARM"})
                    self.get_logger().info("Missão concluída com sucesso. Reiniciando FSM para próxima missão.")
                    self.reset_FSM()

            # =================================================
            # ESTADO NÃO RECONHECIDO
            # =================================================
            case _:
                self.get_logger().error(f"Estado não reconhecido: {current_state}")

    # ==================================================================
    # CHECAGEM ESSENCIAIS DE TÓPICOS
    # ==================================================================

    def check_essential_topics(self):
        """
        Verifica a saúde dos tópicos essenciais.
        Executado por um timer a cada 2 segundos.
        """
        current_time = time.time()
        unhealthy_topics = []
        
        for topic_name, topic_info in self.essencial_topics.items():
            time_since_last = current_time - topic_info["last_received"]
            if time_since_last > self.topic_health_timeout or topic_info["last_received"] == 0.0:
                unhealthy_topics.append(topic_name)
        
        if len(unhealthy_topics) > 0:
            unhealthy_list = ", ".join([f"{name} ({self.essencial_topics[name]['description']})" for name in unhealthy_topics])
            self.get_logger().error(f"Tópicos essenciais inativos detectados: {unhealthy_list}. Resetando FSM....")
            self.reset_FSM()

    def set_state(self, new_state: FSMStateDescription):
        """
        Função central para atualizar o estado da FSM.
        
        Args:
            new_state: Novo estado (FSMStateDescription)
        """
        if self.fsm_state.state != new_state:
            self.fsm_state.state_anterior = self.fsm_state.state
            self.fsm_state.state = new_state
            
            # Limpa variáveis de aguardo ao mudar de estado
            self.fsm_state.waiting_for_state = None
            
            self.get_logger().info(f"NOVO ESTADO FSM: {self.fsm_state.state_anterior.name} -> {self.fsm_state.state.name}")

    def publish_drone_command(self, command_dict):
        """
        Publica um comando para o drone_node usando MissionCommandMSG.
        
        Args:
            command_dict: Dicionário com o comando a ser enviado.
        """
        msg = MissionCommandMSG()
        msg.command = command_dict.get("command", "")
        
        # Parâmetros para GOTO - define NaN se não fornecido (indica "não especificado")
        msg.lat = command_dict.get("lat", float('nan'))
        msg.lon = command_dict.get("lon", float('nan'))
        msg.alt = command_dict.get("alt", float('nan'))
        msg.yaw = command_dict.get("yaw", float('nan'))
        
        # Parâmetros para TAKEOFF
        if "altitude" in command_dict:
            msg.altitude = command_dict["altitude"]
        elif "alt" in command_dict:
            msg.altitude = command_dict["alt"]
        else:
            msg.altitude = float('nan')
        
        # Tipo de inspeção
        msg.inspection_type = command_dict.get("inspection_type", "")

        self.get_logger().info(f"--\> Comando para DroneNode (MissionCommandMSG): {msg.command}")
        self.fsm_command_pub.publish(msg)

    def send_command_and_wait(self, command_dict, expected_state, timeout=10.0):
        """
        Envia comando para o drone_node e configura aguardo de estado.
        
        Args:
            command_dict: Dicionário do comando
            expected_state: Estado esperado do drone após o comando (string)
            timeout: Timeout em segundos
        """
        if self.fsm_state.waiting_for_state is not None:
            self.get_logger().warn(f"Já está aguardando estado {self.fsm_state.waiting_for_state}. Comando ignorado.")
            return False
        
        self.publish_drone_command(command_dict)
        self.fsm_state.waiting_for_state = expected_state
        self.fsm_state.command_sent_time = time.time()
        self.fsm_state.command_timeout = timeout
        self.get_logger().info(f"Comando enviado. Aguardando estado: {expected_state} (timeout: {timeout}s)")
        return True

    def check_waiting_state(self):
        """
        Verifica se o estado esperado foi alcançado após enviar um comando.
        
        Returns:
            str: "waiting", "success" ou "timeout"
        """
        if self.fsm_state.waiting_for_state is None:
            return "success"
        
        if self.drone.state == self.fsm_state.waiting_for_state:
            self.get_logger().info(f"Estado {self.fsm_state.waiting_for_state} alcançado!")
            self.fsm_state.waiting_for_state = None
            return "success"
        
        elapsed = time.time() - self.fsm_state.command_sent_time
        if elapsed > self.fsm_state.command_timeout:
            self.get_logger().error(f"Timeout ({self.fsm_state.command_timeout}s) aguardando estado {self.fsm_state.waiting_for_state}. Estado atual: {self.drone.state}")
            self.fsm_state.waiting_for_state = None
            return "timeout"
        
        return "waiting"

    def send_yaw_command(self, target_yaw):
        """
        Simula comando SET_YAW usando GOTO na posição atual com yaw específico.
        
        Args:
            target_yaw: Yaw alvo em graus (0-360)
        """
        if self.drone.state != "VOANDO_PRONTO":
            self.get_logger().warn(f"Não é possível girar: drone não está VOANDO_PRONTO (estado: {self.drone.state})")
            return False
        
        self.publish_drone_command({
            "command": "GOTO",
            "lat": self.drone.global_position_lat,
            "lon": self.drone.global_position_lon,
            "alt": self.drone.global_position_alt,
            "yaw": target_yaw
        })
        return True

    # ==================================================================
    # CALLBACKS DE EVENTOS
    # ==================================================================

    def dashboard_command_callback(self, msg):
        """
        Recebe comandos do dashboard para controlar a missão.
        """
        import json
        try:
            command_data = json.loads(msg.data)
            command = command_data.get("command", "")
            
            self.get_logger().info(f"<-- Comando do Dashboard recebido (JSON): {msg.data}")
            
            if command == "start_inspection":
                inspection_type = command_data.get("inspection_type", "Flare")
                self.get_logger().info(f"Comando 'start_inspection' recebido do Dashboard. Tipo: {inspection_type}. Verificando condições...")
                if self.fsm_state.state == FSMStateDescription.PRONTO and self.can_execute_mission():
                    self.fsm_state.mission_started = True
                    self.fsm_state.inspection_type = inspection_type
                    self.get_logger().info(f"COMANDO 'START_INSPECTION' ACEITO. Tipo: {inspection_type}. INICIANDO MISSÃO.")
                else:
                    self.get_logger().warn(f"COMANDO 'START_INSPECTION' RECUSADO. Estado atual: {self.fsm_state.state.name}. Condições de execução: {self.can_execute_mission()}")
            
            elif command == "stop_inspection":
                self.get_logger().warn("COMANDO 'STOP_INSPECTION' RECEBIDO. PAUSANDO MISSÃO.")
            
            elif command == "cancel_inspection":
                self.get_logger().warn("COMANDO 'CANCEL_INSPECTION' RECEBIDO. CANCELANDO MISSÃO.")
                self.set_state(FSMStateDescription.RETORNO_RETORNANDO)
            
            elif command == "return_to_base":
                self.get_logger().warn("COMANDO 'RETURN_TO_BASE' RECEBIDO. RETORNANDO À BASE.")
                self.set_state(FSMStateDescription.RETORNO_RETORNANDO)
            
            elif command == "enable_offboard_control_mode":
                self.get_logger().info("COMANDO 'ENABLE_OFFBOARD_CONTROL_MODE' RECEBIDO. Habilitando envio interno no drone_node via drone_commands.")
                self.fsm_state.OffboardControlMode_enabled = True
                self.publish_drone_command({"command": "ENABLE_OFFBOARD"})

            elif command == "abort_mission":
                self.get_logger().warn("COMANDO 'ABORT_MISSION' RECEBIDO. ABORTANDO MISSÃO.")
                self.set_state(FSMStateDescription.RETORNO_RETORNANDO)
            
            elif command == "emergency_land":
                self.get_logger().error("COMANDO 'EMERGENCY_LAND' RECEBIDO. POUSANDO IMEDIATAMENTE.")
                self.publish_drone_command({"command": "LAND"})
            
            else:
                self.get_logger().warn(f"Comando desconhecido recebido: {command}")
                
        except json.JSONDecodeError:
            command_str = msg.data
            self.get_logger().warn(f"Comando recebido não é JSON válido, tentando processar como string: {command_str}")
            
            if command_str.startswith("start_inspection"):
                parts = command_str.split(":", 1)
                inspection_type = parts[1] if len(parts) > 1 else "Flare"
                if self.fsm_state.state == FSMStateDescription.PRONTO and self.can_execute_mission():
                    self.fsm_state.mission_started = True
                    self.fsm_state.inspection_type = inspection_type
                    self.get_logger().info(f"COMANDO 'START_INSPECTION' ACEITO (formato antigo). Tipo: {inspection_type}.")
            elif command_str == "enable_offboard_control_mode":
                self.get_logger().info("COMANDO 'ENABLE_OFFBOARD_CONTROL_MODE' RECEBIDO (formato antigo).")
                self.fsm_state.OffboardControlMode_enabled = True
                self.publish_drone_command({"command": "ENABLE_OFFBOARD"})
            elif command_str == "abort_mission":
                self.set_state(FSMStateDescription.RETORNO_RETORNANDO)
            elif command_str == "emergency_land":
                self.publish_drone_command({"command": "LAND"})

    def drone_state_callback(self, msg: DroneStateMSG):
        """
        Processa estado do drone recebido do drone_node.
        
        Args:
            msg: Mensagem DroneStateMSG do drone_node
        """
        # Atualiza timestamp do tópico essencial
        self.essencial_topics["drone_state"]["last_received"] = time.time()
        
        # Atualiza dados do drone através da classe
        self.drone.update_from_msg(msg)

    def cv_detection_callback(self, msg: CVDetectionMSG):
        """
        Processa detecções do cv_node durante a inspeção.
        
        Args:
            msg: Mensagem CVDetectionMSG do cv_node
        """
        # Só processa detecções durante estados relevantes de inspeção
        current_state = self.fsm_state.state
        if current_state not in [
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO
        ]:
            return
        
        flare_detected_in_frame = False
        
        for detection_item in msg.detections:
            object_type = detection_item.object_type
            confidence = detection_item.confidence
            bbox_center = detection_item.bbox_center if len(detection_item.bbox_center) >= 2 else None
            
            # Processa detecção de flare durante busca
            if object_type == "flare" and current_state == FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO:
                flare_detected_in_frame = True
                self.fsm_state.flare_detected = True
                self.get_logger().info(f"!!! FLARE DETECTADO !!! (confiança: {confidence:.2f})")
                
                if bbox_center:
                    self.fsm_state.target_bbox_center = bbox_center
                    image_center_x = 320.0
                    pixel_offset = bbox_center[0] - image_center_x
                    fov_horizontal = 60.0
                    image_width = 640.0
                    degrees_per_pixel = fov_horizontal / image_width
                    self.fsm_state.target_yaw_offset = pixel_offset * degrees_per_pixel
                    
                    self.get_logger().info(f"Centro do bounding box: {bbox_center}, Offset de yaw: {self.fsm_state.target_yaw_offset:.1f}°")
                
                self.fsm_state.target_centralized = False
                self.set_state(FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO)
                break
            
            # Atualiza centralização durante estado CENTRALIZANDO
            if object_type == "flare" and current_state == FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO and bbox_center:
                image_center_x = 320.0
                pixel_offset = bbox_center[0] - image_center_x
                fov_horizontal = 60.0
                image_width = 640.0
                degrees_per_pixel = fov_horizontal / image_width
                self.fsm_state.target_yaw_offset = pixel_offset * degrees_per_pixel
                self.fsm_state.target_bbox_center = bbox_center
                
                if abs(self.fsm_state.target_yaw_offset) <= self.fsm_state.centralization_tolerance:
                    self.fsm_state.target_centralized = True
                    self.get_logger().info(f"Target centralizado! Offset: {self.fsm_state.target_yaw_offset:.1f}°")
            
            # Processa detecção de anomalia durante escaneamento
            if object_type == "anomalia" and current_state == FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA:
                angle_diff = abs(self.fsm_state.current_inspection_angle - self.fsm_state.last_anomaly_angle)
                if angle_diff > 30 or self.fsm_state.last_anomaly_angle == 0.0:
                    self.fsm_state.anomaly_detected = True
                    self.fsm_state.last_anomaly_angle = self.fsm_state.current_inspection_angle
                    self.get_logger().info(f"!!! ANOMALIA DETECTADA !!! Ângulo: {self.fsm_state.current_inspection_angle:.1f}° (confiança: {confidence:.2f})")
                    self.set_state(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA)
                    break
        
        # Verifica se completou giro de 360° sem detectar flare
        if (current_state == FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO and 
            not flare_detected_in_frame and 
            self.fsm_state.flare_detection_yaw_covered >= 360.0):
            self.get_logger().warn("Flare não detectado após giro completo de 360°.")
            self.set_state(FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA)

    def reset_FSM(self):
        """
        Reseta todas as variáveis de estado para uma nova missão.
        """
        self.drone.reset()
        self.fsm_state.reset()
        
        self.get_logger().info("\033[1m\033[93mMáquina de Estados (FSM) resetada!\033[0m")

    def is_at_waypoint(self, waypoint_lat, waypoint_lon, waypoint_alt):
        """
        Verifica se o drone chegou ao waypoint especificado.
        
        Args:
            waypoint_lat: Latitude do waypoint (graus)
            waypoint_lon: Longitude do waypoint (graus)
            waypoint_alt: Altitude do waypoint (metros)
        
        Returns:
            bool: True se o drone está dentro da tolerância do waypoint
        """
        if (self.drone.global_position_lat == 0.0 or self.drone.global_position_lon == 0.0 or 
            waypoint_lat == 0.0 or waypoint_lon == 0.0):
            return False
        
        lat_diff = abs(self.drone.global_position_lat - waypoint_lat) * 111000
        lon_diff = abs(self.drone.global_position_lon - waypoint_lon) * 111000 * math.cos(math.radians(self.drone.global_position_lat))
        
        distance_2d = math.sqrt(lat_diff**2 + lon_diff**2)
        alt_diff = abs(self.drone.global_position_alt - waypoint_alt)
        distance_3d = math.sqrt(distance_2d**2 + alt_diff**2)
        
        return distance_3d <= self.fsm_state.waypoint_tolerance and not self.drone.on_trajectory

    def can_execute_mission(self):
        """
        Verifica se todas as condições necessárias para iniciar a missão são atendidas.
        """
        current_time = time.time()
        topics_confirmed = True
        
        for topic_name, topic_info in self.essencial_topics.items():
            time_since_last = current_time - topic_info["last_received"]
            if time_since_last > self.topic_health_timeout or topic_info["last_received"] == 0.0:
                topics_confirmed = False
                break
        
        return topics_confirmed


def main(args=None):
    """Função principal do nó."""
    rclpy.init(args=args)
    fsm_node = FSMNode()
    
    try:
        rclpy.spin(fsm_node)
    except KeyboardInterrupt:
        pass
    finally:
        fsm_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
