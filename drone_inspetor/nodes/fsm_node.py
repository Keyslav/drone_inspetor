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
    DashboardFsmCommandMSG,
)


# ==================================================================================================
# CONSTANTES DE AGRUPAMENTO DE ESTADOS DO DRONE
# ==================================================================================================
# Grupos de estados do drone_node para facilitar verificações no FSM

# Estados de movimento GOTO (destino simples)
DRONE_STATES_GOTO = ["VOANDO_GIRANDO_INICIO", "VOANDO_A_CAMINHO", "VOANDO_GIRANDO_FIM"]

# Estados de movimento GOTO_FOCUS (destino com foco em ponto)
DRONE_STATES_GOTO_FOCUS = ["VOANDO_GIRANDO_COM_FOCO", "VOANDO_A_CAMINHO_COM_FOCO"]

# Estados de retorno RTL
DRONE_STATES_RTL = ["RETORNANDO_GIRANDO_INICIO", "RETORNANDO_A_CAMINHO", "RETORNANDO_GIRANDO_FIM"]

# Estados de pouso
DRONE_STATES_POUSANDO = ["POUSANDO"]

# Estados pousado
DRONE_STATES_POUSADO = ["POUSADO_DESARMADO", "POUSADO_ARMADO"]

# Estado fora do modo offboard
DRONE_STATE_OFFBOARD_DESATIVADO = "OFFBOARD_DESATIVADO"

# Todos os estados de movimento ativo (para verificar se drone está em trânsito)
DRONE_STATES_EM_MOVIMENTO = DRONE_STATES_GOTO + DRONE_STATES_GOTO_FOCUS + DRONE_STATES_RTL


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
# ENUM DE COMANDOS DO DASHBOARD PARA FSM
# ==================================================================================================

class DashboardFsmCommandDescription(IntEnum):
    """
    Enum que representa os comandos enviados do Dashboard para o FSM Node.
    Os valores inteiros correspondem ao campo 'command' de DashboardFsmCommandMSG.
    """
    START_INSPECTION = 1           # Iniciar missão de inspeção
    STOP_INSPECTION = 2            # Pausar missão (drone para no ar)
    CANCEL_INSPECTION = 3          # Cancelar missão e retornar à base
    RETURN_TO_BASE = 4             # Retornar à base (RTL)
    ENABLE_OFFBOARD_CONTROL_MODE = 5  # Habilitar modo offboard (compatibilidade)
    ABORT_MISSION = 6              # Abortar missão e retornar à base
    EMERGENCY_LAND = 7             # Pouso de emergência imediato


# ==================================================================================================
# CLASSE DroneStateData
# ==================================================================================================

class DroneStateData:
    """
    Encapsula todos os dados recebidos do drone_node.
    Representa o estado atual do drone conforme reportado pelo drone_node.
    Espelha todos os campos da mensagem DroneStateMSG com nomes idênticos.
    """
    
    def __init__(self):
        """Inicializa todas as variáveis do drone com valores padrão."""
        # --- Estado e Flags ---
        self.state_name = "POUSADO_DESARMADO"
        self.state_duration_sec = 0.0
        self.is_armed = False
        self.is_landed = False
        self.is_on_trajectory = False
        
        # --- Posição Corrente Local (NED) ---
        self.current_local_x = 0.0
        self.current_local_y = 0.0
        self.current_local_z = 0.0
        
        # --- Posição Corrente Global (GPS) ---
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_altitude = 0.0
        
        # --- Orientação Corrente ---
        self.current_yaw_deg = 0.0
        self.current_yaw_deg_normalized = 0.0
        self.current_yaw_rad = 0.0
        
        # --- Posição HOME Global (GPS) ---
        self.home_global_lat = 0.0
        self.home_global_lon = 0.0
        self.home_global_alt = 0.0
        
        # --- Posição HOME Local (NED) ---
        self.home_local_x = 0.0
        self.home_local_y = 0.0
        self.home_local_z = 0.0
        
        # --- Orientação HOME ---
        self.home_yaw_deg = 0.0
        self.home_yaw_deg_normalized = 0.0
        self.home_yaw_rad = 0.0
        
        # --- Posição Alvo Local (NED) ---
        self.target_local_x = 0.0
        self.target_local_y = 0.0
        self.target_local_z = 0.0
        
        # --- Posição Alvo Global (GPS) ---
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.target_alt = 0.0
        
        # --- Orientação Alvo ---
        self.target_direction_yaw_deg = 0.0
        self.target_direction_yaw_deg_normalized = 0.0
        self.target_direction_yaw_rad = 0.0
        self.target_final_yaw_deg = 0.0
        self.target_final_yaw_deg_normalized = 0.0
        self.target_final_yaw_rad = 0.0
        
        # --- Ponto de Foco (para GOTO_FOCUS) ---
        self.focus_lat = 0.0
        self.focus_lon = 0.0
        self.focus_yaw_deg = 0.0
        self.focus_yaw_deg_normalized = 0.0
        self.focus_yaw_rad = 0.0
        
        # --- Última Posição Estática (para hover estável) ---
        self.last_static_position_x = 0.0
        self.last_static_position_y = 0.0
        self.last_static_position_z = 0.0
        self.last_static_yaw_deg = 0.0
        self.last_static_yaw_deg_normalized = 0.0
        self.last_static_yaw_rad = 0.0
    
    def reset(self):
        """Reseta variáveis para valores padrão."""
        self.__init__()
    
    def update_from_msg(self, msg: DroneStateMSG):
        """
        Atualiza valores a partir de mensagem ROS DroneStateMSG.
        Mapeia todos os campos da mensagem para os atributos da classe.
        
        Args:
            msg: Mensagem DroneStateMSG recebida do drone_node
        """
        # --- Estado e Flags ---
        self.state_name = msg.state_name
        self.state_duration_sec = msg.state_duration_sec
        self.is_armed = msg.is_armed
        self.is_landed = msg.is_landed
        self.is_on_trajectory = msg.is_on_trajectory
        
        # --- Posição Corrente Local (NED) ---
        self.current_local_x = msg.current_local_x
        self.current_local_y = msg.current_local_y
        self.current_local_z = msg.current_local_z
        
        # --- Posição Corrente Global (GPS) ---
        self.current_latitude = msg.current_latitude
        self.current_longitude = msg.current_longitude
        self.current_altitude = msg.current_altitude
        
        # --- Orientação Corrente ---
        self.current_yaw_deg = msg.current_yaw_deg
        self.current_yaw_deg_normalized = msg.current_yaw_deg_normalized
        self.current_yaw_rad = msg.current_yaw_rad
        
        # --- Posição HOME Global (GPS) ---
        self.home_global_lat = msg.home_global_lat
        self.home_global_lon = msg.home_global_lon
        self.home_global_alt = msg.home_global_alt
        
        # --- Posição HOME Local (NED) ---
        self.home_local_x = msg.home_local_x
        self.home_local_y = msg.home_local_y
        self.home_local_z = msg.home_local_z
        
        # --- Orientação HOME ---
        self.home_yaw_deg = msg.home_yaw_deg
        self.home_yaw_deg_normalized = msg.home_yaw_deg_normalized
        self.home_yaw_rad = msg.home_yaw_rad
        
        # --- Posição Alvo Local (NED) ---
        self.target_local_x = msg.target_local_x
        self.target_local_y = msg.target_local_y
        self.target_local_z = msg.target_local_z
        
        # --- Posição Alvo Global (GPS) ---
        self.target_lat = msg.target_lat
        self.target_lon = msg.target_lon
        self.target_alt = msg.target_alt
        
        # --- Orientação Alvo ---
        self.target_direction_yaw_deg = msg.target_direction_yaw_deg
        self.target_direction_yaw_deg_normalized = msg.target_direction_yaw_deg_normalized
        self.target_direction_yaw_rad = msg.target_direction_yaw_rad
        self.target_final_yaw_deg = msg.target_final_yaw_deg
        self.target_final_yaw_deg_normalized = msg.target_final_yaw_deg_normalized
        self.target_final_yaw_rad = msg.target_final_yaw_rad
        
        # --- Ponto de Foco (para GOTO_FOCUS) ---
        self.focus_lat = msg.focus_lat
        self.focus_lon = msg.focus_lon
        self.focus_yaw_deg = msg.focus_yaw_deg
        self.focus_yaw_deg_normalized = msg.focus_yaw_deg_normalized
        self.focus_yaw_rad = msg.focus_yaw_rad
        
        # --- Última Posição Estática (para hover estável) ---
        self.last_static_position_x = msg.last_static_position_x
        self.last_static_position_y = msg.last_static_position_y
        self.last_static_position_z = msg.last_static_position_z
        self.last_static_yaw_deg = msg.last_static_yaw_deg
        self.last_static_yaw_deg_normalized = msg.last_static_yaw_deg_normalized
        self.last_static_yaw_rad = msg.last_static_yaw_rad


# ==================================================================================================
# CLASSE FSMState
# ==================================================================================================

class FSMState:
    """
    Encapsula todas as variáveis de estado da FSM.
    Contém o estado atual e todas as variáveis de controle da missão.
    """
    
    # Mapeamento de comandos do dashboard para estados permitidos
    VALID_DASHBOARD_COMMANDS = {
        DashboardFsmCommandDescription.START_INSPECTION: [
            FSMStateDescription.PRONTO,
        ],
        DashboardFsmCommandDescription.STOP_INSPECTION: [
            FSMStateDescription.EXECUTANDO_ARMANDO,
            FSMStateDescription.EXECUTANDO_DECOLANDO,
            FSMStateDescription.EXECUTANDO_VOANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO,
        ],
        DashboardFsmCommandDescription.CANCEL_INSPECTION: [
            FSMStateDescription.EXECUTANDO_ARMANDO,
            FSMStateDescription.EXECUTANDO_DECOLANDO,
            FSMStateDescription.EXECUTANDO_VOANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA,
            FSMStateDescription.INSPECAO_FINALIZADA,
        ],
        DashboardFsmCommandDescription.RETURN_TO_BASE: [
            FSMStateDescription.EXECUTANDO_VOANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA,
            FSMStateDescription.INSPECAO_FINALIZADA,
        ],
        DashboardFsmCommandDescription.ENABLE_OFFBOARD_CONTROL_MODE: [
            # Comando de compatibilidade, permitido em qualquer estado
            FSMStateDescription.DESATIVADO,
            FSMStateDescription.PRONTO,
        ],
        DashboardFsmCommandDescription.ABORT_MISSION: [
            # Abortar missão permitido em qualquer estado de execução
            FSMStateDescription.EXECUTANDO_ARMANDO,
            FSMStateDescription.EXECUTANDO_DECOLANDO,
            FSMStateDescription.EXECUTANDO_VOANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA,
            FSMStateDescription.INSPECAO_FINALIZADA,
            FSMStateDescription.RETORNO_RETORNANDO,
        ],
        DashboardFsmCommandDescription.EMERGENCY_LAND: [
            # Emergência permitida em QUALQUER estado (prioridade absoluta)
            FSMStateDescription.DESATIVADO,
            FSMStateDescription.PRONTO,
            FSMStateDescription.EXECUTANDO_ARMANDO,
            FSMStateDescription.EXECUTANDO_DECOLANDO,
            FSMStateDescription.EXECUTANDO_VOANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA,
            FSMStateDescription.INSPECAO_FINALIZADA,
            FSMStateDescription.RETORNO_RETORNANDO,
            FSMStateDescription.RETORNO_DESARMANDO,
        ],
    }
    
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
        
        # Variáveis de inspeção
        self.inspection_completed = False
        self.inspection_start_time = 0.0
        self.inspection_start_angle = 0.0    # Ângulo inicial da órbita de inspeção (graus)
        self.current_inspection_angle = 0.0  # Ângulo atual na órbita de inspeção (graus)
        self.last_anomaly_angle = 0.0
        self.failure_pause_start_time = 0.0
        self.inspection_pause_start_time = 0.0
        self.anomaly_focus_start_time = 0.0
    
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
        
        self.inspection_completed = False
        self.inspection_start_time = 0.0
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
        
        msg.inspection_completed = self.inspection_completed
        msg.inspection_start_angle = self.inspection_start_angle
        msg.current_inspection_angle = self.current_inspection_angle
        msg.last_anomaly_angle = self.last_anomaly_angle
        msg.failure_pause_start_time = self.failure_pause_start_time
        msg.inspection_pause_start_time = self.inspection_pause_start_time
        
        self._node.fsm_state_pub.publish(msg)
    
    def muda_estado(self, new_state: FSMStateDescription):
        """
        Função central para atualizar o estado da FSM.
        Verifica se há realmente uma mudança de estado antes de atualizar.
        
        Args:
            new_state: Novo estado (FSMStateDescription)
        """
        if self.state != new_state:
            self.state_anterior = self.state
            self.state = new_state
            
            # Limpa variáveis de aguardo ao mudar de estado
            self.waiting_for_state = None
            
            self._node.get_logger().info(f"MUDANÇA DE ESTADO FSM: {self.state_anterior.name} -> {self.state.name}")
    
    def verifica_validade_do_comando(self, command: int) -> tuple[bool, str, DashboardFsmCommandDescription | None]:
        """
        Valida se um comando do dashboard é permitido no estado atual.
        Similar ao verifica_validade_do_comando do DroneState no drone_node.
        
        Args:
            command: Código inteiro do comando recebido na mensagem
            
        Returns:
            tuple: (can_execute, error_message, command_enum)
                - can_execute: True se comando é permitido, False se não
                - error_message: Mensagem de erro se não permitido, string vazia se permitido
                - command_enum: O enum do comando se válido, None se inválido
        """
        # Primeiro, tenta converter o inteiro para o enum
        try:
            command_enum = DashboardFsmCommandDescription(command)
        except ValueError:
            return False, f"Comando desconhecido: {command}", None
        
        # Verifica se o comando é permitido no estado atual
        allowed_states = self.VALID_DASHBOARD_COMMANDS.get(command_enum, [])
        
        if not allowed_states:
            return False, f"Comando '{command_enum.name}' não está configurado em VALID_DASHBOARD_COMMANDS", command_enum
        
        if self.state in allowed_states:
            return True, "", command_enum
        else:
            allowed_names = [s.name for s in allowed_states]
            return False, (
                f"Comando '{command_enum.name}' não permitido no estado {self.state.name}. "
                f"Estados permitidos: {allowed_names}"
            ), command_enum


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

        # QoS para comandos críticos: RELIABLE + TRANSIENT_LOCAL garante entrega
        # IMPORTANTE: Deve corresponder ao QoS do subscriber em drone_node
        qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
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
        self.fsm_state = FSMState(self)
        
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
            DashboardFsmCommandMSG, "/drone_inspetor/interno/dashboard_node/fsm_commands", self.dashboard_fsm_command_callback, qos_commands)
        
        # Recebe detecções do cv_node
        self.cv_detection_sub = self.create_subscription(
            CVDetectionMSG, "/drone_inspetor/interno/cv_node/object_detections", self.cv_detection_callback, qos_sensor_data)

        # ==================================================================
        # TIMERS
        # ==================================================================
        # Timer principal da FSM - executa a lógica de estados a cada 100ms
        self.fsm_timer = self.create_timer(0.1, self.verifica_mudanca_de_estado_fsm)
        
        # Timer para publicação periódica do estado FSM
        self.fsm_state_pub_timer = self.create_timer(0.5, lambda: self.fsm_state.publish())
        
        # Timer para verificação de saúde dos tópicos essenciais - executa a cada 2 segundos
        self.topic_health_timer = self.create_timer(2.0, self.check_essential_topics)
        
        self.get_logger().info("================ FSM NODE PRONTO ================")

    # ==================================================================
    # LÓGICA PRINCIPAL DA FSM
    # ==================================================================

    def verifica_mudanca_de_estado_fsm(self):
        """
        O coração da FSM, executado a cada 100ms.
        Implementa a máquina de estados completa usando match/case.
        """
        # =================================================
        # VERIFICAÇÃO DE TÓPICOS ESSENCIAIS (PRIMEIRO PASSO OBRIGATÓRIO)
        # Garante que estamos recebendo dados dos tópicos essenciais
        # antes de processar qualquer lógica da FSM
        # =================================================
        if not self.verifica_topicos_essenciais():
            if current_state != FSMStateDescription.DESATIVADO:
                self.reset_FSM()
                self.fsm_state.muda_estado(FSMStateDescription.DESATIVADO)
        
        # =================================================
        # CÓPIAS LOCAIS PARA EVITAR RACE CONDITIONS
        # Armazena o estado atual no início para garantir
        # consistência durante toda a execução do step
        # =================================================
        current_state = self.fsm_state.state
        fsm_mission_started = self.fsm_state.mission_started
        
        # Variáveis do drone usadas para decisões críticas
        drone_state_name = self.drone.state_name
        drone_current_yaw_deg = self.drone.current_yaw_deg

        # Verifica se o drone saiu do modo OFFBOARD (via estado do drone)
        if current_state != FSMStateDescription.DESATIVADO:
            if drone_state_name == DRONE_STATE_OFFBOARD_DESATIVADO:
                self.get_logger().error(f"Drone saiu do modo OFFBOARD (estado: {drone_state_name}). Resetando FSM...")
                self.reset_FSM()
                return

        # =================================================
        # MÁQUINA DE ESTADOS USANDO MATCH/CASE
        # =================================================
        match current_state:

            # =================================================
            # ESTADO: DESATIVADO
            # =================================================
            case FSMStateDescription.DESATIVADO:
                if drone_state_name == DRONE_STATE_OFFBOARD_DESATIVADO:
                    self.get_logger().info(f"Estado: DESATIVADO >>> Aguardando mudança para Modo OFFBOARD!", throttle_duration_sec=5)
                    return

                self.get_logger().info("Estado: DESATIVADO >>> Sistema no modo OFFBOARD. Transicionando para Estado PRONTO!")
                self.fsm_state.muda_estado(FSMStateDescription.PRONTO)
                return
            
            # =================================================
            # ESTADO: PRONTO
            # =================================================
            case FSMStateDescription.PRONTO:
                if fsm_mission_started:
                    self.get_logger().info("\033[1m\033[31mINICIANDO MISSÃO: Iniciando sequência de missão!\033[0m")
                    self.fsm_state.mission_started = False
                    self.get_logger().info(f"INICIANDO MISSÃO: Tipo de inspeção selecionada: {self.fsm_state.inspection_type}")
                    self.get_logger().info("INICIANDO MISSÃO: Transicionando para estado EXECUTANDO_ARMANDO...")
                    self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_ARMANDO)
                    return
                
                self.get_logger().info("Estado PRONTO: Aguardando Missão...", throttle_duration_sec=5)

            # =================================================
            # ESTADO: EXECUTANDO_ARMANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_ARMANDO:
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Aguardando drone armar... Estado atual: {drone_state_name}", throttle_duration_sec=2)
                    return
                elif wait_status == "timeout":
                    self.get_logger().error("Timeout ao armar! Abortando missão.")
                    self.fsm_state.muda_estado(FSMStateDescription.DESATIVADO)
                    return

                if drone_state_name == "POUSADO_ARMADO":
                    self.get_logger().info("Drone ARMADO confirmado. Transicionando para DECOLANDO...")
                    self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_DECOLANDO)
                    return

                if drone_state_name == "POUSADO_DESARMADO":
                    self.get_logger().info("Enviando comando ARM e aguardando confirmação...")
                    self.send_command_and_wait({"command": "ARM"}, "POUSADO_ARMADO", timeout=10.0)
                    return
                
                self.get_logger().warn(f"Estado inesperado do drone durante ARMANDO: {drone_state_name}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_DECOLANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_DECOLANDO:
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Aguardando decolagem... Estado: {drone_state_name}", throttle_duration_sec=2)
                    return
                elif wait_status == "timeout":
                    self.get_logger().error("Timeout na decolagem! Abortando missão.")
                    self.fsm_state.muda_estado(FSMStateDescription.DESATIVADO)
                    return

                if drone_state_name == "VOANDO_PRONTO":
                    self.get_logger().info("Decolagem concluída! Drone em VOANDO_PRONTO. Transicionando para VOANDO...")
                    self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_VOANDO)
                    return

                if drone_state_name == "POUSADO_ARMADO":
                    self.get_logger().info(f"Enviando comando TAKEOFF para {self.fsm_state.takeoff_altitude}m...")
                    self.send_command_and_wait(
                        {"command": "TAKEOFF", "alt": self.fsm_state.takeoff_altitude},
                        "VOANDO_PRONTO",
                        timeout=30.0
                    )
                    return
                
                if drone_state_name == "VOANDO_DECOLANDO":
                    self.get_logger().info("Drone decolando...", throttle_duration_sec=2)
                    return
                
                self.get_logger().warn(f"Estado inesperado durante DECOLANDO: {drone_state_name}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_VOANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_VOANDO:
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Voando para ponto de inspeção... Estado: {drone_state_name}", throttle_duration_sec=2)
                    return

                if drone_state_name == "VOANDO_PRONTO":
                    drone_reached_waypoint = self.is_at_waypoint(
                        self.fsm_state.inspection_point_lat,
                        self.fsm_state.inspection_point_lon,
                        self.fsm_state.inspection_point_alt
                    )

                    if drone_reached_waypoint:
                        self.get_logger().info("Drone chegou ao ponto de inspeção. Transicionando para INSPECIONANDO_DETECTANDO...")
                        self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO)
                        return

                    self.get_logger().info(f"Navegando para ponto de inspeção ({self.fsm_state.inspection_point_lat:.6f}, {self.fsm_state.inspection_point_lon:.6f}, {self.fsm_state.inspection_point_alt:.1f})...")
                    self.send_command_and_wait(
                        {"command": "GOTO", "lat": self.fsm_state.inspection_point_lat, "lon": self.fsm_state.inspection_point_lon, "alt": self.fsm_state.inspection_point_alt},
                        "VOANDO_PRONTO",
                        timeout=60.0
                    )
                    return
                
                # Estados de trajetória GOTO (destino simples)
                if drone_state_name in DRONE_STATES_GOTO:
                    self.get_logger().info(f"Em trajetória GOTO... Estado: {drone_state_name}", throttle_duration_sec=2)
                    return
                
                # Estados de trajetória GOTO_FOCUS (destino com foco)
                if drone_state_name in DRONE_STATES_GOTO_FOCUS:
                    self.get_logger().info(f"Em trajetória GOTO_FOCUS... Estado: {drone_state_name}", throttle_duration_sec=2)
                    return
                
                # Estado de decolagem ainda em progresso
                if drone_state_name == "VOANDO_DECOLANDO":
                    self.get_logger().info("Ainda decolando...", throttle_duration_sec=2)
                    return
                
                self.get_logger().warn(f"Estado inesperado durante VOANDO: {drone_state_name}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_DETECTANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO:
                if not self.fsm_state.flare_detected:
                    if self.fsm_state.flare_detection_start_yaw is None:
                        self.fsm_state.flare_detection_start_yaw = drone_current_yaw_deg
                        self.fsm_state.flare_detection_current_target_yaw = drone_current_yaw_deg
                        self.fsm_state.flare_detection_yaw_covered = 0.0
                        self.fsm_state.flare_detection_yaw_reached = False
                        self.get_logger().info(f"Iniciando busca incremental de flare a partir do yaw {self.fsm_state.flare_detection_start_yaw:.1f}°...")
                    
                    if self.fsm_state.flare_detection_current_target_yaw is not None:
                        yaw_diff_to_target = drone_current_yaw_deg - self.fsm_state.flare_detection_current_target_yaw
                        if yaw_diff_to_target < -180:
                            yaw_diff_to_target += 360
                        if yaw_diff_to_target > 180:
                            yaw_diff_to_target -= 360
                        
                        if abs(yaw_diff_to_target) <= 1.0:
                            if not self.fsm_state.flare_detection_yaw_reached:
                                self.fsm_state.flare_detection_yaw_reached = True
                                self.get_logger().info(f"Yaw alvo {self.fsm_state.flare_detection_current_target_yaw:.1f}° alcançado. Aguardando detecção...")
                        else:
                            self.get_logger().debug(f"Aguardando alcançar yaw {self.fsm_state.flare_detection_current_target_yaw:.1f}° (atual: {drone_current_yaw_deg:.1f}°)", 
                                                   throttle_duration_sec=1)
                            return
                    
                    if self.fsm_state.flare_detection_yaw_reached:
                        yaw_diff = drone_current_yaw_deg - self.fsm_state.flare_detection_start_yaw
                        if yaw_diff < -180:
                            yaw_diff += 360
                        if yaw_diff > 180:
                            yaw_diff -= 360
                        
                        self.fsm_state.flare_detection_yaw_covered = abs(yaw_diff)
                        
                        if self.fsm_state.flare_detection_yaw_covered >= 360.0:
                            self.get_logger().warn("Flare não detectado após busca completa de 360°.")
                            self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA)
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
                        target_yaw = (drone_current_yaw_deg + self.fsm_state.target_yaw_offset) % 360.0
                        yaw_diff = drone_current_yaw_deg - target_yaw
                        if yaw_diff < -180:
                            yaw_diff += 360
                        if yaw_diff > 180:
                            yaw_diff -= 360
                        
                        if abs(yaw_diff) <= self.fsm_state.centralization_tolerance:
                            self.fsm_state.target_centralized = True
                            self.get_logger().info("Target centralizado! Transicionando para próximo estado...")
                        else:
                            self.get_logger().info(f"Centralizando target... Offset: {self.fsm_state.target_yaw_offset:.1f}° (atual: {drone_current_yaw_deg:.1f}°, alvo: {target_yaw:.1f}°)", 
                                                  throttle_duration_sec=0.5)
                            self.send_yaw_command(target_yaw)
                    else:
                        self.get_logger().warn("Aguardando cálculo do offset de centralização...")
                
                if self.fsm_state.target_centralized:
                    self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA:
                if not self.fsm_state.inspection_completed:
                    if self.fsm_state.inspection_start_time == 0.0:
                        self.fsm_state.inspection_start_time = time.time()
                        self.get_logger().warn("Voo orbital não implementado. Simulando escaneamento por 5 segundos...")
                    
                    elapsed = time.time() - self.fsm_state.inspection_start_time
                    if elapsed >= 5.0:
                        self.get_logger().info("Escaneamento simulado concluído. Finalizando inspeção.")
                        self.fsm_state.inspection_completed = True
                        self.fsm_state.muda_estado(FSMStateDescription.INSPECAO_FINALIZADA)
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
                    self.fsm_state.muda_estado(FSMStateDescription.INSPECAO_FINALIZADA)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA:
                self.get_logger().info("Anomalia detectada durante escaneamento!")
                self.fsm_state.anomaly_focus_start_time = time.time()
                self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_FOCANDO)

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
                    self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_SEM_ANOMALIA)

            # =================================================
            # ESTADO: INSPECAO_FINALIZADA
            # =================================================
            case FSMStateDescription.INSPECAO_FINALIZADA:
                if self.fsm_state.inspection_pause_start_time == 0.0:
                    self.fsm_state.inspection_pause_start_time = time.time()
                    self.get_logger().info("Inspeção finalizada. Pausando por 3 segundos...")
                
                if time.time() - self.fsm_state.inspection_pause_start_time >= 3.0:
                    self.get_logger().info("Pausa concluída. Iniciando retorno ao helideck.")
                    self.fsm_state.muda_estado(FSMStateDescription.RETORNO_RETORNANDO)

            # =================================================
            # ESTADO: RETORNO_RETORNANDO
            # =================================================
            case FSMStateDescription.RETORNO_RETORNANDO:
                # Verifica se drone já pousou
                if drone_state_name in DRONE_STATES_POUSADO:
                    self.get_logger().info("Drone pousou. Transicionando para DESARMANDO...")
                    self.fsm_state.muda_estado(FSMStateDescription.RETORNO_DESARMANDO)
                    return
                
                # Verifica se está pousando
                if drone_state_name in DRONE_STATES_POUSANDO:
                    self.get_logger().info("Drone pousando...", throttle_duration_sec=2)
                    return
                
                # Verifica se está em trânsito RTL
                if drone_state_name in DRONE_STATES_RTL:
                    self.get_logger().info(f"RTL em progresso... Estado: {drone_state_name}", throttle_duration_sec=2)
                    return
                
                # Verifica se está pronto para voar (precisa enviar RTL)
                if drone_state_name == "VOANDO_PRONTO":
                    self.get_logger().info("Enviando comando RTL...")
                    self.publish_drone_command({"command": "RTL"})
                    return
                
                # Ainda em algum movimento anterior (GOTO por exemplo)
                if drone_state_name in DRONE_STATES_GOTO + DRONE_STATES_GOTO_FOCUS:
                    self.get_logger().info(f"Aguardando fim do movimento atual ({drone_state_name}) antes de RTL...", throttle_duration_sec=2)
                    return
                
                self.get_logger().warn(f"Estado inesperado durante RETORNO_RETORNANDO: {drone_state_name}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: RETORNO_DESARMANDO
            # =================================================
            case FSMStateDescription.RETORNO_DESARMANDO:
                # PX4 desarma automaticamente após pouso (COM_DISARM_LAND)
                # Verifica se já desarmou para finalizar missão
                if drone_state_name == "POUSADO_DESARMADO":
                    self.get_logger().info("Drone desarmado pelo PX4. Missão concluída com sucesso!")
                    self.reset_FSM()
                    self.fsm_state.muda_estado(FSMStateDescription.PRONTO)
                else:
                    self.get_logger().info("Aguardando desarme automático do PX4...", throttle_duration_sec=2)

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
        
        if self.drone.state_name == self.fsm_state.waiting_for_state:
            self.get_logger().info(f"Estado {self.fsm_state.waiting_for_state} alcançado!")
            self.fsm_state.waiting_for_state = None
            return "success"
        
        elapsed = time.time() - self.fsm_state.command_sent_time
        if elapsed > self.fsm_state.command_timeout:
            self.get_logger().error(f"Timeout ({self.fsm_state.command_timeout}s) aguardando estado {self.fsm_state.waiting_for_state}. Estado atual: {self.drone.state_name}")
            self.fsm_state.waiting_for_state = None
            return "timeout"
        
        return "waiting"

    def send_yaw_command(self, target_yaw):
        """
        Simula comando SET_YAW usando GOTO na posição atual com yaw específico.
        
        Args:
            target_yaw: Yaw alvo em graus (0-360)
        """
        if self.drone.state_name != "VOANDO_PRONTO":
            self.get_logger().warn(f"Não é possível girar: drone não está VOANDO_PRONTO (estado: {self.drone.state_name})")
            return False
        
        self.publish_drone_command({
            "command": "GOTO",
            "lat": self.drone.current_latitude,
            "lon": self.drone.current_longitude,
            "alt": self.drone.current_altitude,
            "yaw": target_yaw
        })
        return True

    # ==================================================================
    # CALLBACKS DE EVENTOS
    # ==================================================================

    def dashboard_fsm_command_callback(self, msg: DashboardFsmCommandMSG):
        """
        Processa comandos recebidos do dashboard para controlar a missão.
        Este é o callback principal que traduz comandos do Dashboard em ações da FSM.
        
        Similar ao fsm_command_callback do drone_node para consistência.
        
        Args:
            msg: Mensagem DashboardFsmCommandMSG do dashboard_node
        """
        command = msg.command
        
        self.get_logger().info(f"<-- Comando do Dashboard recebido (DashboardFsmCommandMSG): {command}")
        
        # Valida se o comando é permitido no estado atual
        can_execute, error_msg, command_enum = self.fsm_state.verifica_validade_do_comando(command)
        if not can_execute:
            self.get_logger().warn(f"Comando rejeitado: {error_msg}")
            return
        
        # Processa comandos usando match/case (Python 3.10+)
        match command_enum:
            
            case DashboardFsmCommandDescription.START_INSPECTION:
                inspection_type = msg.inspection_type if msg.inspection_type else "Flare"
                self.fsm_state.mission_started = True
                self.fsm_state.inspection_type = inspection_type
                self.get_logger().info(f"Comando 'START_INSPECTION' ACEITO. Tipo: {inspection_type}. INICIANDO MISSÃO.")
            
            case DashboardFsmCommandDescription.STOP_INSPECTION:
                self.get_logger().warn("Comando 'STOP_INSPECTION'. PAUSANDO MISSÃO.")
                if self.drone.state_name in DRONE_STATES_EM_MOVIMENTO:
                    self.get_logger().info("Drone em movimento. Enviando STOP para parar no ar...")
                    self.publish_drone_command({"command": "STOP"})
            
            case DashboardFsmCommandDescription.CANCEL_INSPECTION:
                self.get_logger().warn("Comando 'CANCEL_INSPECTION'. CANCELANDO MISSÃO.")
                if self.drone.state_name in DRONE_STATES_EM_MOVIMENTO:
                    self.get_logger().info("Drone em movimento. Enviando STOP antes de RTL...")
                    self.publish_drone_command({"command": "STOP"})
                self.fsm_state.muda_estado(FSMStateDescription.RETORNO_RETORNANDO)
            
            case DashboardFsmCommandDescription.RETURN_TO_BASE:
                self.get_logger().warn("Comando 'RETURN_TO_BASE'. RETORNANDO À BASE.")
                if self.drone.state_name in DRONE_STATES_EM_MOVIMENTO:
                    self.publish_drone_command({"command": "STOP"})
                self.fsm_state.muda_estado(FSMStateDescription.RETORNO_RETORNANDO)
            
            case DashboardFsmCommandDescription.ENABLE_OFFBOARD_CONTROL_MODE:
                self.get_logger().info("Comando 'ENABLE_OFFBOARD_CONTROL_MODE'. (Offboard é gerenciado pelo drone_node)")
            
            case DashboardFsmCommandDescription.ABORT_MISSION:
                self.get_logger().warn("Comando 'ABORT_MISSION'. ABORTANDO MISSÃO.")
                if self.drone.state_name in DRONE_STATES_EM_MOVIMENTO:
                    self.publish_drone_command({"command": "STOP"})
                self.fsm_state.muda_estado(FSMStateDescription.RETORNO_RETORNANDO)
            
            case DashboardFsmCommandDescription.EMERGENCY_LAND:
                self.get_logger().error("Comando 'EMERGENCY_LAND'. POUSANDO IMEDIATAMENTE.")
                self.publish_drone_command({"command": "LAND"})
            
            case _:
                self.get_logger().warn(f"Comando não reconhecido: {command}")

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
                self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO)
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
                    self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO_ANOMALIA)
                    break
        
        # Verifica se completou giro de 360° sem detectar flare
        if (current_state == FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO and 
            not flare_detected_in_frame and 
            self.fsm_state.flare_detection_yaw_covered >= 360.0):
            self.get_logger().warn("Flare não detectado após giro completo de 360°.")
            self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA)

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
        if (self.drone.current_latitude == 0.0 or self.drone.current_longitude == 0.0 or 
            waypoint_lat == 0.0 or waypoint_lon == 0.0):
            return False
        
        lat_diff = abs(self.drone.current_latitude - waypoint_lat) * 111000
        lon_diff = abs(self.drone.current_longitude - waypoint_lon) * 111000 * math.cos(math.radians(self.drone.current_latitude))
        
        distance_2d = math.sqrt(lat_diff**2 + lon_diff**2)
        alt_diff = abs(self.drone.current_altitude - waypoint_alt)
        distance_3d = math.sqrt(distance_2d**2 + alt_diff**2)
        
        return distance_3d <= self.fsm_state.waypoint_tolerance and not self.drone.is_on_trajectory

    def verifica_topicos_essenciais(self):
        """
        Verifica se está recebendo dados dos tópicos essenciais corretamente.
        
        Esta verificação é executada sempre no início de verifica_mudanca_de_estado_fsm()
        para garantir que a FSM só processe estados quando os tópicos essenciais estão ativos.
        
        Atualmente verifica apenas o DroneStateData, mas futuramente verificará
        múltiplos tópicos essenciais.
        
        Returns:
            bool: True se todos os tópicos essenciais estão sendo recebidos, False caso contrário.
                  Se retornar False, a FSM é resetada automaticamente.
        """
        current_time = time.time()
        
        for topic_name, topic_info in self.essencial_topics.items():
            time_since_last = current_time - topic_info["last_received"]
            
            # Verifica se nunca recebeu dados
            if topic_info["last_received"] == 0.0:
                self.get_logger().warn(
                    f"TÓPICO ESSENCIAL INATIVO: '{topic_name}' ({topic_info['description']}) "
                    f"nunca recebeu dados. Aguardando conexão...",
                    throttle_duration_sec=5
                )
                return False
            
            # Verifica se os dados estão desatualizados
            if time_since_last > self.topic_health_timeout:
                self.get_logger().error(
                    f"TÓPICO ESSENCIAL FALHOU: '{topic_name}' ({topic_info['description']}) "
                    f"não recebe dados há {time_since_last:.1f}s (timeout: {self.topic_health_timeout}s). "
                    f"Resetando FSM...",
                    throttle_duration_sec=5
                )
                self.reset_FSM()
                return False
        
        return True


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
