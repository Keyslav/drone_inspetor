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
# ENUM DE ESTADOS DO DRONE (Replicado do drone_node.py)
# ==================================================================================================

class DroneStateDescription(IntEnum):
    """
    Enum unificado que representa todos os estados possíveis do drone.
    Réplica do DroneStateDescription em drone_node.py para interpretação da mensagem.
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
# CONSTANTES DE AGRUPAMENTO DE ESTADOS DO DRONE
# ==================================================================================================
# Grupos de estados do drone_node para facilitar verificações no FSM

# Estados de movimento GOTO (destino simples)
DRONE_STATES_GOTO = [
    DroneStateDescription.VOANDO_GIRANDO_INICIO,
    DroneStateDescription.VOANDO_A_CAMINHO,
    DroneStateDescription.VOANDO_GIRANDO_FIM
]

# Estados de movimento GOTO_FOCUS (destino com foco em ponto)
DRONE_STATES_GOTO_FOCUS = [
    DroneStateDescription.VOANDO_GIRANDO_COM_FOCO,
    DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO
]

# Estados de retorno RTL
DRONE_STATES_RTL = [
    DroneStateDescription.RETORNANDO_GIRANDO_INICIO,
    DroneStateDescription.RETORNANDO_A_CAMINHO,
    DroneStateDescription.RETORNANDO_GIRANDO_FIM
]

# Estados de pouso
DRONE_STATES_POUSANDO = [DroneStateDescription.POUSANDO]

# Estados pousado
DRONE_STATES_POUSADO = [
    DroneStateDescription.POUSADO_DESARMADO,
    DroneStateDescription.POUSADO_ARMADO
]

# Todos os estados de movimento ativo (para verificar se drone está em trânsito)
DRONE_STATES_EM_MOVIMENTO = DRONE_STATES_GOTO + DRONE_STATES_GOTO_FOCUS + DRONE_STATES_RTL



# ==================================================================================================
# ENUM DE ESTADOS DA FSM - UNIFICADO
# ==================================================================================================

class FSMStateDescription(IntEnum):
    """
    Enum unificado que representa todos os estados possíveis da FSM.
    """
    # Estados de nível 0 - Sistema
    DESATIVADO = 0                              # Sistema desativado, aguardando tópicos essenciais
    PRONTO = 1                                  # Pronto para iniciar missão
    
    # Estados EXECUTANDO_MISSAO - Nível 1
    EXECUTANDO_ARMANDO = 10                     # Armando motores
    EXECUTANDO_DECOLANDO = 11                   # Decolagem em progresso
    EXECUTANDO_INSPECIONANDO = 12               # Percorrendo pontos de inspeção
    
    # Estados INSPECIONANDO - Nível 2
    EXECUTANDO_INSPECIONANDO_DETECTANDO = 20    # Girando 360° procurando target
    EXECUTANDO_INSPECIONANDO_CENTRALIZANDO = 21 # Centralizando target no frame
    EXECUTANDO_INSPECIONANDO_ESCANEANDO = 22    # Escaneando/inspecionando
    EXECUTANDO_INSPECIONANDO_FALHA = 25         # Falha na detecção do target
    
    # Estados de finalização
    INSPECAO_FINALIZADA = 30                    # Pausa pós-inspeção
    RETORNANDO = 40                             # Retornando ao home (RTL)


# ==================================================================================================
# ENUM DE COMANDOS DO DASHBOARD PARA FSM
# ==================================================================================================

class DashboardFsmCommandDescription(IntEnum):
    """
    Enum que representa os comandos enviados do Dashboard para o FSM Node.
    Os valores inteiros correspondem ao campo 'command' de DashboardFsmCommandMSG.
    """
    INICIAR_MISSAO = 1     # Iniciar missão (aceito apenas em PRONTO)
    CANCELAR_MISSAO = 2    # Cancelar missão e retornar (aceito em EXECUTANDO_*)


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
        self.state = DroneStateDescription.POUSADO_DESARMADO
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
    
    
    def update_from_msg(self, msg: DroneStateMSG):
        """
        Atualiza valores a partir de mensagem ROS DroneStateMSG.
        Mapeia todos os campos da mensagem para os atributos da classe.
        
        Args:
            msg: Mensagem DroneStateMSG recebida do drone_node
        """
        # --- Estado e Flags ---
        try:
            self.state = DroneStateDescription(msg.state)
        except ValueError:
            # Estado desconhecido, mantém o atual
            pass
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
        DashboardFsmCommandDescription.INICIAR_MISSAO: [
            FSMStateDescription.PRONTO,
        ],
        DashboardFsmCommandDescription.CANCELAR_MISSAO: [
            FSMStateDescription.EXECUTANDO_ARMANDO,
            FSMStateDescription.EXECUTANDO_DECOLANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA,
            FSMStateDescription.INSPECAO_FINALIZADA,
        ],
    }
    
    def __init__(self, node: 'FSMNode'):
        """
        Inicializa todas as variáveis de estado da FSM.
        
        Args:
            node: Referência ao FSMNode para acesso ao publisher
        """
        import json
        import os
        
        self._node = node
        
        # Estado atual
        self.state = FSMStateDescription.DESATIVADO
        
        # Controle de aguardo de comando
        self.waiting_for_state = None
        self.command_timeout = 10.0
        self.command_sent_time = 0.0
        
        # Flags de missão (controladas pelos comandos do dashboard)
        self.on_mission = False           # Setada por INICIAR_MISSAO
        self.cancel_mission = False       # Setada por CANCELAR_MISSAO
        
        # Carrega missões disponíveis do arquivo JSON
        self.missions = {}
        self.mission = None               # Missão atual selecionada
        self._load_missions()
        
        # Parâmetros de configuração
        self.takeoff_altitude = 20.0
        self.waypoint_tolerance = 0.5
        self.tempo_de_permanencia = 5.0   # Tempo de permanência em cada ponto (segundos)
        
        # Controle de waypoints da missão
        self.current_waypoint_index = 0
        self.waypoint_arrival_time = 0.0
    
    def _load_missions(self):
        """Carrega missões do arquivo missions.json."""
        import json
        import os
        from ament_index_python.packages import get_package_share_directory
        
        # Caminho do arquivo de missões usando o diretório share do pacote ROS2
        package_share_dir = get_package_share_directory('drone_inspetor')
        missions_file = os.path.join(package_share_dir, 'missions', 'missions.json')
        
        try:
            with open(missions_file, 'r') as f:
                self.missions = json.load(f)
            self._node.get_logger().info(f"Missões carregadas: {list(self.missions.keys())}")
        except FileNotFoundError:
            self._node.get_logger().error(f"Arquivo de missões não encontrado: {missions_file}")
            self.missions = {}
        except json.JSONDecodeError as e:
            self._node.get_logger().error(f"Erro ao parsear missions.json: {e}")
            self.missions = {}
    
    def valida_missao(self, mission_name: str) -> tuple[bool, str]:
        """
        Valida se uma missão existe e pode ser carregada.
        
        Args:
            mission_name: Nome da missão a validar
            
        Returns:
            tuple: (is_valid, error_message)
        """
        if not mission_name:
            return False, "Nome da missão não especificado"
        
        if mission_name not in self.missions:
            available = list(self.missions.keys())
            return False, f"Missão '{mission_name}' não encontrada. Disponíveis: {available}"
        
        # Carrega a missão
        self.mission = self.missions[mission_name]
        self.current_waypoint_index = 0
        
        self._node.get_logger().info(f"Missão '{mission_name}' carregada com sucesso")
        return True, ""
    
    def reset(self):
        """
        Reseta variáveis para nova missão.
        Mantém valores de configuração (altitudes, tolerâncias, etc.).
        """
        self.state = FSMStateDescription.DESATIVADO
        
        self.waiting_for_state = None
        self.command_sent_time = 0.0
        self.command_timeout = 10.0
        
        # Flags de missão
        self.on_mission = False
        self.cancel_mission = False
        
        # Missão atual
        self.mission = None
        self.current_waypoint_index = 0
        self.waypoint_arrival_time = 0.0
        
        # Loga
        self._node.get_logger().info("\033[1m\033[93mMáquina de Estados (FSM) resetada!\033[0m")
    
    def publish(self):
        """
        Publica estado atual no tópico ROS.
        Cria mensagem FSMState e publica através do publisher do node.
        """
        msg = FSMStateMSG()
        msg.state = int(self.state)
        msg.state_name = self.state.name
        
        msg.waiting_for_state = self.waiting_for_state.name if self.waiting_for_state else ""
        msg.command_timeout = self.command_timeout
        msg.command_sent_time = self.command_sent_time
        
        # Flags de missão
        msg.on_mission = self.on_mission
        msg.cancel_mission = self.cancel_mission
        
        # Missão atual
        msg.mission_name = self.mission.get('nome', '') if self.mission else ""
        msg.tempo_de_permanencia = self.tempo_de_permanencia
        
        # Parâmetros de configuração
        msg.takeoff_altitude = self.takeoff_altitude
        msg.waypoint_tolerance = self.waypoint_tolerance
        
        # Controle de waypoints
        msg.current_waypoint_index = self.current_waypoint_index
        msg.waypoint_arrival_time = self.waypoint_arrival_time
        
        self._node.fsm_state_pub.publish(msg)
    
    def muda_estado(self, new_state: FSMStateDescription):
        """
        Função central para atualizar o estado da FSM.
        Verifica se há realmente uma mudança de estado antes de atualizar.
        
        Args:
            new_state: Novo estado (FSMStateDescription)
        """
        if self.state != new_state:
            old_state = self.state
            self.state = new_state
            
            # Limpa variáveis de aguardo ao mudar de estado
            self.waiting_for_state = None
            
            self._node.get_logger().info(f"MUDANÇA DE ESTADO FSM: {old_state.name} -> {self.state.name}")
    
    def verifica_validade_do_comando(self, command_enum: DashboardFsmCommandDescription) -> tuple[bool, str]:
        """
        Valida se um comando do dashboard é permitido no estado atual.
        Similar ao verifica_validade_do_comando do DroneState no drone_node.
        
        Args:
            command_enum: Enum do comando já convertido
            
        Returns:
            tuple: (can_execute, error_message)
                - can_execute: True se comando é permitido, False se não
                - error_message: Mensagem de erro se não permitido, string vazia se permitido
        """
        # Verifica se o comando é permitido no estado atual
        allowed_states = self.VALID_DASHBOARD_COMMANDS.get(command_enum, [])
        
        if not allowed_states:
            return False, f"Comando '{command_enum.name}' não está configurado em VALID_DASHBOARD_COMMANDS"
        
        if self.state in allowed_states:
            return True, ""
        else:
            allowed_names = [s.name for s in allowed_states]
            return False, (
                f"Comando '{command_enum.name}' não permitido no estado {self.state.name}. "
                f"Estados permitidos: {allowed_names}"
            )
    
    def verifica_mudanca_de_estado(self):
        """
        O coração da FSM, executado a cada 100ms.
        Implementa a máquina de estados completa usando match/case.
        """
        # =================================================
        # CÓPIAS LOCAIS PARA EVITAR RACE CONDITIONS
        # =================================================
        current_state = self.state
        on_mission = self.on_mission
        cancel_mission = self.cancel_mission
        
        # Variáveis do drone usadas para decisões críticas
        drone_state = self._node.drone.state
        drone_current_yaw_deg = self._node.drone.current_yaw_deg

        # Verifica se o drone saiu do modo OFFBOARD (via estado do drone)
        if current_state != FSMStateDescription.DESATIVADO:
            if drone_state == DroneStateDescription.OFFBOARD_DESATIVADO:
                self._node.get_logger().error(f"Drone saiu do modo OFFBOARD (estado: {drone_state.name}). Resetando FSM...")
                self.reset()
                return
        
        # Verifica flag cancel_mission (prioridade sobre on_mission)
        if cancel_mission:
            self._node.get_logger().warn("Flag cancel_mission detectada. Resetando on_mission e transicionando para RETORNANDO...")
            self.on_mission = False
            self.cancel_mission = False
            self.muda_estado(FSMStateDescription.RETORNANDO)
            return

        # =================================================
        # MÁQUINA DE ESTADOS USANDO MATCH/CASE
        # =================================================
        match current_state:

            # =================================================
            # ESTADO: DESATIVADO
            # =================================================
            case FSMStateDescription.DESATIVADO:
                if drone_state == DroneStateDescription.OFFBOARD_DESATIVADO:
                    self._node.get_logger().info(f"Estado da FSM: DESATIVADO >>> Aguardando mudança do Drone para Modo OFFBOARD!", throttle_duration_sec=5)
                    return

                if drone_state == DroneStateDescription.POUSADO_DESARMADO:
                    self._node.get_logger().info("Estado da FSM: DESATIVADO >>> Drone no modo OFFBOARD, POUSADO e DESARMADO. Transicionando para Estado PRONTO!")
                    self.muda_estado(FSMStateDescription.PRONTO)
                    return

                self._node.get_logger().info("Estado da FSM: DESATIVADO >>> Drone no modo OFFBOARD. Aguardando mudança do Drone para Modo POUSADO e DESARMADO!", throttle_duration_sec=5)
                return


            # =================================================
            # ESTADO: PRONTO
            # =================================================
            case FSMStateDescription.PRONTO:

                if drone_state != DroneStateDescription.POUSADO_DESARMADO:
                    self._node.get_logger().warn("Drone não está POUSADO e DESARMADO. Resetando FSM...")
                    self.muda_estado(FSMStateDescription.DESATIVADO)
                    self.reset()
                    return

                if on_mission:
                    mission_name = self.mission['nome']
                    self._node.get_logger().info(f"\033[1m\033[31mINICIANDO MISSÃO: {mission_name}\033[0m")
                    self._node.get_logger().info("INICIANDO MISSÃO: Transicionando para estado EXECUTANDO_ARMANDO...")
                    self.muda_estado(FSMStateDescription.EXECUTANDO_ARMANDO)
                    return
                
                self._node.get_logger().info("Estado da FSM: PRONTO >>> Aguardando Missão...", throttle_duration_sec=5)

            # =================================================
            # ESTADO: EXECUTANDO_ARMANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_ARMANDO:
                wait_status = self._node.check_waiting_state()
                if wait_status == "waiting":
                    self._node.get_logger().info(f"Aguardando drone armar... Estado atual: {drone_state.name}", throttle_duration_sec=2)
                    return
                elif wait_status == "timeout":
                    self._node.get_logger().error("Timeout ao armar! Abortando missão.")
                    self.muda_estado(FSMStateDescription.DESATIVADO)
                    return

                if drone_state == DroneStateDescription.POUSADO_ARMADO:
                    self._node.get_logger().info("Drone ARMADO confirmado. Transicionando para DECOLANDO...")
                    self.muda_estado(FSMStateDescription.EXECUTANDO_DECOLANDO)
                    return

                if drone_state == DroneStateDescription.POUSADO_DESARMADO:
                    self._node.get_logger().info("Enviando comando ARM e aguardando confirmação...")
                    self._node.send_command_and_wait({"command": "ARM"}, DroneStateDescription.POUSADO_ARMADO, timeout=10.0)
                    return
                
                self._node.get_logger().warn(f"Estado inesperado do drone durante ARMANDO: {drone_state.name}", throttle_duration_sec=2)
                self.muda_estado(FSMStateDescription.DESATIVADO)
                self.reset()
                return

            # =================================================
            # ESTADO: EXECUTANDO_DECOLANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_DECOLANDO:
                wait_status = self._node.check_waiting_state()
                if wait_status == "waiting":
                    self._node.get_logger().info(f"Aguardando decolagem... Estado: {drone_state.name}", throttle_duration_sec=2)
                    return
                elif wait_status == "timeout":
                    self._node.get_logger().error("Timeout na decolagem! Abortando missão.")
                    self.muda_estado(FSMStateDescription.DESATIVADO)
                    return

                if drone_state == DroneStateDescription.VOANDO_PRONTO:
                    self._node.get_logger().info("Decolagem concluída! Drone em VOANDO_PRONTO. Transicionando para INSPECIONANDO...")
                    self.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO)
                    return

                if drone_state == DroneStateDescription.POUSADO_ARMADO:
                    self._node.get_logger().info(f"Enviando comando TAKEOFF para {self.takeoff_altitude}m...")
                    self._node.send_command_and_wait(
                        {"command": "TAKEOFF", "alt": self.takeoff_altitude},
                        DroneStateDescription.VOANDO_PRONTO,
                        timeout=30.0
                    )
                    return
                
                if drone_state == DroneStateDescription.VOANDO_DECOLANDO:
                    self._node.get_logger().info("Drone decolando...", throttle_duration_sec=2)
                    return
                
                self._node.get_logger().warn(f"Estado inesperado durante DECOLANDO: {drone_state.name}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO:
                wait_status = self._node.check_waiting_state()
                if wait_status == "waiting":
                    self._node.get_logger().info(f"Aguardando chegada ao ponto... Estado: {drone_state.name}", throttle_duration_sec=2)
                    return

                # Verifica se há missão carregada
                if not self.mission or 'pontos_de_inspecao' not in self.mission:
                    self._node.get_logger().error("Nenhuma missão carregada ou pontos de inspeção não definidos!")
                    self.muda_estado(FSMStateDescription.RETORNANDO)
                    return

                pontos = self.mission['pontos_de_inspecao']
                
                # Verifica se todos os pontos foram percorridos
                if self.current_waypoint_index >= len(pontos):
                    self._node.get_logger().info("Todos os pontos de inspeção percorridos!")
                    self.muda_estado(FSMStateDescription.INSPECAO_FINALIZADA)
                    return

                ponto_atual = pontos[self.current_waypoint_index]

                # Se drone está VOANDO_PRONTO
                if drone_state == DroneStateDescription.VOANDO_PRONTO:
                    # Se waypoint_arrival_time > 0, significa que chegamos ao ponto
                    if self.waypoint_arrival_time > 0.0:
                        # Verifica se é ponto de detecção (precisa aguardar tempo_de_permanencia)
                        if ponto_atual.get('ponto_de_deteccao', False):
                            elapsed = time.time() - self.waypoint_arrival_time
                            if elapsed < self.tempo_de_permanencia:
                                remaining = self.tempo_de_permanencia - elapsed
                                self._node.get_logger().info(f"Aguardando no ponto de detecção... {remaining:.1f}s restantes", throttle_duration_sec=1)
                                return
                        
                        # Próximo ponto
                        self._node.get_logger().info(f"Ponto {self.current_waypoint_index + 1}/{len(pontos)} concluído!")
                        self.current_waypoint_index += 1
                        self.waypoint_arrival_time = 0.0
                        return
                    
                    # Enviar comando para o ponto atual
                    cmd = ponto_atual.get('command', 'GOTO')
                    self._node.get_logger().info(f"Navegando para ponto {self.current_waypoint_index + 1}/{len(pontos)} ({cmd})...")
                    
                    if cmd == 'GOTO':
                        self._node.send_command_and_wait(
                            {"command": "GOTO", "lat": ponto_atual['lat'], "lon": ponto_atual['lon'], 
                             "alt": ponto_atual['alt'], "yaw": ponto_atual.get('yaw', 0.0)},
                            DroneStateDescription.VOANDO_PRONTO,
                            timeout=60.0
                        )
                    elif cmd == 'GOTO_FOCUS':
                        self._node.send_command_and_wait(
                            {"command": "GOTO_FOCUS", "lat": ponto_atual['lat'], "lon": ponto_atual['lon'], 
                             "alt": ponto_atual['alt'], "focus_lat": ponto_atual['focus_lat'], 
                             "focus_lon": ponto_atual['focus_lon']},
                            DroneStateDescription.VOANDO_PRONTO,
                            timeout=60.0
                        )
                    
                    # Marca tempo de chegada (será validado quando voltar para VOANDO_PRONTO)
                    self.waypoint_arrival_time = time.time()
                    return
                
                # Estados de trajetória GOTO (destino simples)
                if drone_state in DRONE_STATES_GOTO:
                    self._node.get_logger().info(f"Em trajetória GOTO... Estado: {drone_state.name}", throttle_duration_sec=2)
                    return
                
                # Estados de trajetória GOTO_FOCUS (destino com foco)
                if drone_state in DRONE_STATES_GOTO_FOCUS:
                    self._node.get_logger().info(f"Em trajetória GOTO_FOCUS... Estado: {drone_state.name}", throttle_duration_sec=2)
                    return
                
                # Estado de decolagem ainda em progresso
                if drone_state == DroneStateDescription.VOANDO_DECOLANDO:
                    self._node.get_logger().info("Ainda decolando...", throttle_duration_sec=2)
                    return
                
                self._node.get_logger().warn(f"Estado inesperado durante INSPECIONANDO: {drone_state.name}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_DETECTANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO:
                # Por enquanto, não faz nada (reservado para futura implementação)
                pass

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_CENTRALIZANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_CENTRALIZANDO:
                # Por enquanto, não faz nada (reservado para futura implementação)
                pass

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO:
                # Por enquanto, não faz nada (reservado para futura implementação)
                pass

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_FALHA
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA:
                # Por enquanto, não faz nada (reservado para futura implementação)
                pass

            # =================================================
            # ESTADO: INSPECAO_FINALIZADA
            # =================================================
            case FSMStateDescription.INSPECAO_FINALIZADA:
                self._node.get_logger().info("Inspeção finalizada. Iniciando retorno...")
                self.muda_estado(FSMStateDescription.RETORNANDO)

            # =================================================
            # ESTADO: RETORNANDO
            # =================================================
            case FSMStateDescription.RETORNANDO:
                # Verifica se drone já pousou e desarmou
                if drone_state == DroneStateDescription.POUSADO_DESARMADO:
                    self._node.get_logger().info("Drone pousou e desarmou. Missão concluída com sucesso!")
                    self.reset()
                    self.muda_estado(FSMStateDescription.PRONTO)
                    return
                
                # Verifica se drone pousou mas ainda armado
                if drone_state == DroneStateDescription.POUSADO_ARMADO:
                    self._node.get_logger().info("Drone pousou. Aguardando desarme automático do PX4...", throttle_duration_sec=2)
                    return
                
                # Verifica se está pousando
                if drone_state in DRONE_STATES_POUSANDO:
                    self._node.get_logger().info("Drone pousando...", throttle_duration_sec=2)
                    return
                
                # Verifica se está em trânsito RTL
                if drone_state in DRONE_STATES_RTL:
                    self._node.get_logger().info(f"RTL em progresso... Estado: {drone_state.name}", throttle_duration_sec=2)
                    return
                
                # Verifica se está pronto para voar (precisa enviar RTL)
                if drone_state == DroneStateDescription.VOANDO_PRONTO:
                    self._node.get_logger().info("Enviando comando RTL...")
                    self._node.publish_drone_command({"command": "RTL"})
                    return
                
                # Ainda em algum movimento anterior (GOTO por exemplo)
                if drone_state in DRONE_STATES_GOTO + DRONE_STATES_GOTO_FOCUS:
                    self._node.get_logger().info(f"Aguardando fim do movimento atual ({drone_state.name}) antes de RTL...", throttle_duration_sec=2)
                    return
                
                self._node.get_logger().warn(f"Estado inesperado durante RETORNANDO: {drone_state.name}", throttle_duration_sec=2)

            # =================================================
            # ESTADO NÃO RECONHECIDO
            # =================================================
            case _:
                self._node.get_logger().error(f"Estado não reconhecido: {current_state}")


# ==================================================================================================
# CLASSE FSMNode
# ==================================================================================================

class FSMNode(Node):
    """
    Implementa a lógica de estados hierárquica da missão de inspeção.
    
    FLUXO DA MISSÃO:
    1. DESATIVADO: Falha em algum dos tópicos essenciais
    2. PRONTO: Drone pronto e aguardando comando de missão do dashboard
    3. EXECUTANDO_ARMANDO: Arma os motores
    4. EXECUTANDO_DECOLANDO: Decola para altitude de missão
    5. EXECUTANDO_INSPECIONANDO: Percorre pontos de inspeção da missão
    6. INSPECAO_FINALIZADA: Transição para retorno
    7. RETORNANDO: RTL, pousa e desarma
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
        self.fsm_timer = self.create_timer(0.1, lambda: self.fsm_state.verifica_mudanca_de_estado())
        
        # Timer para publicação periódica do estado FSM
        self.fsm_state_pub_timer = self.create_timer(0.5, lambda: self.fsm_state.publish())
        
        # Timer para verificação de saúde dos tópicos essenciais - executa a cada 2 segundos
        self.topic_health_timer = self.create_timer(2.0, self.check_essential_topics)
        
        self.get_logger().info("================ FSM NODE PRONTO ================")

    # ==================================================================
    # CHECAGEM ESSENCIAIS DE TÓPICOS
    # ==================================================================

    def check_essential_topics(self) -> bool:
        """
        Verifica a saúde dos tópicos essenciais.
        Executado por um timer a cada 2 segundos E chamado em verifica_mudanca_de_estado.
        
        Se algum tópico essencial estiver inativo:
        - Loga o erro
        - Envia comando STOP para o drone
        - Reseta a FSM
        
        Returns:
            bool: True se todos os tópicos essenciais estão saudáveis, False caso contrário.
        """
        current_time = time.time()
        unhealthy_topics = []
        
        for topic_name, topic_info in self.essencial_topics.items():
            time_since_last = current_time - topic_info["last_received"]
            
            # Verifica se nunca recebeu ou está desatualizado
            if topic_info["last_received"] == 0.0:
                unhealthy_topics.append((topic_name, "nunca recebeu dados"))
            elif time_since_last > self.topic_health_timeout:
                unhealthy_topics.append((topic_name, f"sem dados há {time_since_last:.1f}s"))
        
        if unhealthy_topics:
            unhealthy_list = ", ".join([
                f"{name} ({self.essencial_topics[name]['description']}): {reason}" 
                for name, reason in unhealthy_topics
            ])
            self.get_logger().error(
                f"TÓPICOS ESSENCIAIS INATIVOS: {unhealthy_list}. Enviando STOP e resetando FSM...",
                throttle_duration_sec=5
            )
            
            # Envia STOP para parar o drone imediatamente
            self.publish_drone_command({"command": "STOP"})
            
            # Reseta a FSM
            self.fsm_state.reset()
            
            return False
        
        return True

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

        self.get_logger().info(f"---> Comando para DroneNode (MissionCommandMSG): {msg.command}")
        self.fsm_command_pub.publish(msg)


    def send_command_and_wait(self, command_dict, expected_state, timeout=10.0):
        """
        Envia comando para o drone_node e configura aguardo de estado.
        
        Args:
            command_dict: Dicionário do comando
            expected_state: Estado esperado do drone após o comando (DroneStateDescription)
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
            self.get_logger().info(f"Estado {self.fsm_state.waiting_for_state.name} alcançado!")
            self.fsm_state.waiting_for_state = None
            return "success"
        
        elapsed = time.time() - self.fsm_state.command_sent_time
        if elapsed > self.fsm_state.command_timeout:
            self.get_logger().error(f"Timeout ({self.fsm_state.command_timeout}s) aguardando estado {self.fsm_state.waiting_for_state.name}. Estado atual: {self.drone.state.name}")
            self.fsm_state.waiting_for_state = None
            return "timeout"
        
        return "waiting"

    def send_yaw_command(self, target_yaw):
        """
        Simula comando SET_YAW usando GOTO na posição atual com yaw específico.
        
        Args:
            target_yaw: Yaw alvo em graus (0-360)
        """
        if self.drone.state != DroneStateDescription.VOANDO_PRONTO:
            self.get_logger().warn(f"Não é possível girar: drone não está VOANDO_PRONTO (estado: {self.drone.state.name})")
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
        
        Comandos suportados:
        - INICIAR_MISSAO: Seta on_mission=True e carrega missão
        - CANCELAR_MISSAO: Seta cancel_mission=True para tratamento no verifica_mudanca_de_estado
        
        Args:
            msg: Mensagem DashboardFsmCommandMSG do dashboard_node
        """
        command = msg.command
        
        # Converte o inteiro para o enum
        try:
            command_enum = DashboardFsmCommandDescription(command)
        except ValueError:
            self.get_logger().warn(f"Comando do Dashboard desconhecido recebido: {command}")
            return
        
        self.get_logger().info(f"<-- Comando do Dashboard recebido (DashboardFsmCommandMSG): {command_enum.name}")
        
        # Valida se o comando é permitido no estado atual
        can_execute, error_msg = self.fsm_state.verifica_validade_do_comando(command_enum)
        
        if not can_execute:
            self.get_logger().warn(f"Comando rejeitado: {error_msg}")
            return
        
        # Processa comandos usando match/case
        match command_enum:
            
            case DashboardFsmCommandDescription.INICIAR_MISSAO:
                mission_name = msg.mission
                
                # Valida e carrega a missão
                is_valid, error = self.fsm_state.valida_missao(mission_name)
                if not is_valid:
                    self.get_logger().error(f"Missão inválida: {error}")
                    return
                
                # Seta flag on_mission para ser tratada no verifica_mudanca_de_estado
                self.fsm_state.on_mission = True
                self.get_logger().info(f"Comando 'INICIAR_MISSAO' ACEITO. Missão: {mission_name}")
            
            case DashboardFsmCommandDescription.CANCELAR_MISSAO:
                self.get_logger().warn("Comando 'CANCELAR_MISSAO'")
                
                # Seta flag cancel_mission para ser tratada no verifica_mudanca_de_estado
                self.fsm_state.cancel_mission = True
                


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
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO
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
            if object_type == "anomalia" and current_state == FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO:
                angle_diff = abs(self.fsm_state.current_inspection_angle - self.fsm_state.last_anomaly_angle)
                if angle_diff > 30 or self.fsm_state.last_anomaly_angle == 0.0:
                    self.fsm_state.anomaly_detected = True
                    self.fsm_state.last_anomaly_angle = self.fsm_state.current_inspection_angle
                    self.get_logger().info(f"!!! ANOMALIA DETECTADA !!! Ângulo: {self.fsm_state.current_inspection_angle:.1f}° (confiança: {confidence:.2f})")
                    self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO)
                    break
        
        # Verifica se completou giro de 360° sem detectar flare
        if (current_state == FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO and 
            not flare_detected_in_frame and 
            self.fsm_state.flare_detection_yaw_covered >= 360.0):
            self.get_logger().warn("Flare não detectado após giro completo de 360°.")
            self.fsm_state.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA)



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
