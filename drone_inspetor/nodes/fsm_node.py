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
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
import time
import math # Para cálculos de ângulos e distâncias
from enum import IntEnum

# Importações das mensagens ROS customizadas
from drone_inspetor_msgs.msg import (
    DroneStateMSG,
    FSMStateMSG,
    CVDetectionMSG,
    CVDetectionItemMSG,
    DashboardFsmCommandMSG,
)
from drone_inspetor_msgs.action import DroneCommand
from drone_inspetor_msgs.srv import CVDetectionSRV, RecordDetectionsSRV, EnableAnomalyDetectionSRV


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
    EXECUTANDO_INSPECIONANDO_DETECTANDO = 20    # Aguardando detecção do objeto alvo
    EXECUTANDO_INSPECIONANDO_ESCANEANDO = 22    # Escaneando anomalias (detecção habilitada)
    EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO = 23  # Finalizando escaneamento
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
        self.state = DroneStateDescription.OFFBOARD_DESATIVADO
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
        
        # Flags de missão (controladas pelos comandos do dashboard)
        self.on_mission = False           # Setada por INICIAR_MISSAO
        self.cancel_mission = False       # Setada por CANCELAR_MISSAO
        
        # Carrega missões disponíveis do arquivo JSON
        self.missions = {}
        self.mission = None               # Missão atual selecionada
        self._load_missions()
        
        # Parâmetros de configuração
        self.takeoff_altitude = 20.0
        self.tempo_de_permanencia = 5.0   # Tempo de permanência em cada ponto (segundos)
        
        # Controle de waypoints da missão
        self.ponto_de_inspecao_indice_atual = 0
        self.ponto_de_inspecao_tempo_de_chegada = 0.0
        self._detection_started = False  # Flag para controlar se a detecção já foi iniciada neste ponto
        self._detection_start_time = 0.0  # Timestamp de início da detecção
        self._scanning_started = False  # Flag para controlar se o escaneamento já foi iniciado
        self._scanning_start_time = 0.0  # Timestamp de início do escaneamento
        
        # Diretório da missão atual (criado quando missão inicia)
        self.mission_folder_path = ""
    
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
        self.ponto_de_inspecao_indice_atual = 0
        
        # Cria pasta da missão
        self.mission_folder_path = self._create_mission_folder(mission_name)
        
        self._node.get_logger().info(f"Missão '{mission_name}' carregada com sucesso")
        self._node.get_logger().info(f"Pasta da missão: {self.mission_folder_path}")
        return True, ""
    
    def _create_mission_folder(self, mission_name: str) -> str:
        """
        Cria a pasta da missão com formato mission_<data>_<hora>.
        
        Args:
            mission_name: Nome da missão (para log)
            
        Returns:
            Caminho completo da pasta da missão
        """
        import os
        from datetime import datetime
        
        # Obtém diretório base do parâmetro global (via node)
        missions_directory = self._node.get_missions_directory()
        
        # Expande ~ para o home do usuário
        missions_directory = os.path.expanduser(missions_directory)
        
        # Cria nome da pasta com timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"mission_{timestamp}"
        folder_path = os.path.join(missions_directory, folder_name)
        
        # Cria o diretório (e subpastas necessárias)
        try:
            os.makedirs(folder_path, exist_ok=True)
            # Cria subpasta para fotos
            os.makedirs(os.path.join(folder_path, "fotos"), exist_ok=True)
            # Cria subpasta para vídeos
            os.makedirs(os.path.join(folder_path, "videos"), exist_ok=True)
            self._node.get_logger().info(f"📁 Pasta da missão criada: {folder_path}")
        except OSError as e:
            self._node.get_logger().error(f"Erro ao criar pasta da missão: {e}")
            return ""
        
        return folder_path
    
    def reset(self):
        """
        Reseta variáveis para nova missão.
        Mantém valores de configuração (altitudes, tolerâncias, etc.).
        Cancela qualquer action em andamento.
        """
        # Cancela action em andamento, se houver
        self._node.cancel_current_action()
        
        self.state = FSMStateDescription.DESATIVADO
        
        # Flags de missão
        self.on_mission = False
        self.cancel_mission = False
        
        # Missão atual
        self.mission = None
        self.mission_folder_path = ""
        self.ponto_de_inspecao_indice_atual = 0
        self.ponto_de_inspecao_tempo_de_chegada = 0.0
        
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
        
        # Flags de missão
        msg.on_mission = self.on_mission
        msg.cancel_mission = self.cancel_mission
        
        # Missão atual
        msg.mission_name = self.mission.get('nome', '') if self.mission else ""
        msg.mission_folder_path = self.mission_folder_path
        msg.tempo_de_permanencia = self.tempo_de_permanencia
        
        # Parâmetros de configuração
        msg.takeoff_altitude = self.takeoff_altitude
        
        # Controle de waypoints
        msg.ponto_de_inspecao_indice_atual = self.ponto_de_inspecao_indice_atual
        
        # Total de pontos de inspeção
        if self.mission and 'pontos_de_inspecao' in self.mission:
            msg.total_pontos_de_inspecao = len(self.mission['pontos_de_inspecao'])
        else:
            msg.total_pontos_de_inspecao = 0
        
        msg.ponto_de_inspecao_tempo_de_chegada = self.ponto_de_inspecao_tempo_de_chegada
        
        # Objeto alvo e tipos de anomalia do ponto atual (se disponível)
        msg.objeto_alvo = ""
        msg.tipos_anomalia = []
        if self.mission and 'pontos_de_inspecao' in self.mission:
            pontos = self.mission['pontos_de_inspecao']
            if 0 <= self.ponto_de_inspecao_indice_atual < len(pontos):
                ponto_atual = pontos[self.ponto_de_inspecao_indice_atual]
                msg.objeto_alvo = ponto_atual.get('objeto_alvo', '')
                msg.tipos_anomalia = ponto_atual.get('tipos_anomalia', [])
        
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
            self._node.cancel_current_action()
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
                    self._node.get_logger().info(f"Estado da FSM: DESATIVADO >>> Aguardando mudança do Drone para Modo OFFBOARD!", throttle_duration_sec=20)
                    return

                if drone_state == DroneStateDescription.POUSADO_DESARMADO:
                    self._node.get_logger().info("Estado da FSM: DESATIVADO >>> Drone no modo OFFBOARD, POUSADO e DESARMADO. Transicionando para Estado PRONTO!")
                    self.muda_estado(FSMStateDescription.PRONTO)
                    return

                self._node.get_logger().info("Estado da FSM: DESATIVADO >>> Drone no modo OFFBOARD. Aguardando mudança do Drone para Modo POUSADO e DESARMADO!", throttle_duration_sec=20)
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
                
                self._node.get_logger().info("Estado da FSM: PRONTO >>> Aguardando Missão...", throttle_duration_sec=500)

            # =================================================
            # ESTADO: EXECUTANDO_ARMANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_ARMANDO:
                if self._node._action_in_progress:
                    # Verifica timeout da action
                    if self._node.check_action_timeout():
                        self._node.get_logger().error("Timeout ao armar! Abortando missão.")
                        self.muda_estado(FSMStateDescription.DESATIVADO)
                        return
                    self._node.get_logger().info(f"Aguardando drone armar... Estado atual: {drone_state.name}", throttle_duration_sec=2)
                    return

                if drone_state == DroneStateDescription.POUSADO_ARMADO:
                    self._node.get_logger().info("Drone ARMADO confirmado. Transicionando para DECOLANDO...")
                    self.muda_estado(FSMStateDescription.EXECUTANDO_DECOLANDO)
                    return

                if drone_state == DroneStateDescription.POUSADO_DESARMADO:
                    self._node.get_logger().info("Enviando comando ARM e aguardando confirmação...")
                    self._node.send_drone_action({"command": "ARM"})
                    return
                
                self._node.get_logger().warn(f"Estado inesperado do drone durante ARMANDO: {drone_state.name}", throttle_duration_sec=2)
                self.muda_estado(FSMStateDescription.DESATIVADO)
                self.reset()
                return

            # =================================================
            # ESTADO: EXECUTANDO_DECOLANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_DECOLANDO:
                if self._node._action_in_progress:
                    # Verifica timeout da action
                    if self._node.check_action_timeout():
                        self._node.get_logger().error("Timeout na decolagem! Abortando missão.")
                        self.muda_estado(FSMStateDescription.DESATIVADO)
                        return
                    self._node.get_logger().info(f"Aguardando decolagem... Estado: {drone_state.name}", throttle_duration_sec=2)
                    return

                if drone_state == DroneStateDescription.VOANDO_PRONTO:
                    self._node.get_logger().info("Decolagem concluída! Drone em VOANDO_PRONTO. Transicionando para INSPECIONANDO...")
                    self.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO)
                    return

                if drone_state == DroneStateDescription.POUSADO_ARMADO:
                    self._node.get_logger().info(f"Enviando comando TAKEOFF para {self.takeoff_altitude}m...")
                    self._node.send_drone_action(
                        {"command": "TAKEOFF", "alt": self.takeoff_altitude}
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
                
                # Se já existe uma ação em andamento, verifica timeout e aguarda
                if self._node._action_in_progress:
                    if self._node.check_action_timeout():
                        self._node.get_logger().error("Timeout no GOTO! Abortando missão.")
                        self.muda_estado(FSMStateDescription.RETORNANDO)
                        return
                    self._node.get_logger().info(f"Aguardando chegada ao ponto... Estado: {drone_state.name}", throttle_duration_sec=20)
                    return

                # Verifica se há missão carregada
                if not self.mission or 'pontos_de_inspecao' not in self.mission:
                    self._node.get_logger().error("Nenhuma missão carregada ou pontos de inspeção não definidos!")
                    self.muda_estado(FSMStateDescription.RETORNANDO)
                    return

                pontos = self.mission['pontos_de_inspecao']
                
                # Verifica se todos os pontos foram percorridos
                if self.ponto_de_inspecao_indice_atual >= len(pontos):
                    self._node.get_logger().info("Todos os pontos de inspeção percorridos!")
                    self.muda_estado(FSMStateDescription.INSPECAO_FINALIZADA)
                    return

                ponto_atual = pontos[self.ponto_de_inspecao_indice_atual]

                # Se chegamos aqui, não há action em progresso.
                # Verificamos se chegamos ao ponto (ponto_de_inspecao_tempo_de_chegada > 0)
                # O ponto_de_inspecao_tempo_de_chegada é setado no _action_result_callback quando obtém sucesso.
                
                if self.ponto_de_inspecao_tempo_de_chegada > 0.0:
                    # Chegamos e a action terminou com sucesso.
                    
                    # Verifica se é ponto de detecção
                    if ponto_atual.get('ponto_de_deteccao', False):
                        # Transiciona para estado de detecção
                        self._node.get_logger().info(
                            f"Chegou ao ponto de detecção {self.ponto_de_inspecao_indice_atual + 1}. "
                            f"Transitando para EXECUTANDO_INSPECIONANDO_DETECTANDO..."
                        )
                        self.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO)
                        return
                    
                    # Não era ponto de detecção, avança para o próximo ponto
                    self._node.get_logger().info(f"Ponto {self.ponto_de_inspecao_indice_atual + 1}/{len(pontos)} concluído!")
                    self.ponto_de_inspecao_indice_atual += 1
                    self.ponto_de_inspecao_tempo_de_chegada = 0.0
                    return
                
                else:
                    # Se não estamos esperando action E arrival_time é 0, precisamos ir para o ponto.
                    # (Ou falhou anteriormente, e vamos tentar de novo)
                    
                    cmd = ponto_atual.get('command', 'GOTO')
                    self._node.get_logger().info(f"Navegando para ponto {self.ponto_de_inspecao_indice_atual + 1}/{len(pontos)} ({cmd})...")
                    
                    if cmd == 'GOTO':
                        self._node.send_drone_action(
                            {"command": "GOTO", "lat": ponto_atual['lat'], "lon": ponto_atual['lon'], 
                             "alt": ponto_atual['alt'], "yaw": ponto_atual.get('yaw', 0.0)}
                        )
                    elif cmd == 'GOTO_FOCUS':
                        self._node.send_drone_action(
                            {"command": "GOTO_FOCUS", "lat": ponto_atual['lat'], "lon": ponto_atual['lon'], 
                             "alt": ponto_atual['alt'], "focus_lat": ponto_atual['focus_lat'], 
                             "focus_lon": ponto_atual['focus_lon']}
                        )
                    return

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_DETECTANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO:
                # Obtém dados da missão e ponto atual
                if not self.mission or 'pontos_de_inspecao' not in self.mission:
                    self._node.get_logger().error("Missão não disponível no estado DETECTANDO!")
                    self.muda_estado(FSMStateDescription.RETORNANDO)
                    return
                
                pontos = self.mission['pontos_de_inspecao']
                if self.ponto_de_inspecao_indice_atual >= len(pontos):
                    self._node.get_logger().error("Índice de ponto inválido no estado DETECTANDO!")
                    self.muda_estado(FSMStateDescription.RETORNANDO)
                    return
                
                ponto_atual = pontos[self.ponto_de_inspecao_indice_atual]
                
                # Lê informações de detecção do ponto atual
                objeto_alvo = ponto_atual.get('objeto_alvo', '')
                tipos_anomalia = ponto_atual.get('tipos_anomalia', [])
                
                # === FASE 1: Iniciar detecção (uma vez ao entrar no estado) ===
                if not self._detection_started:
                    # Marca que a detecção foi iniciada
                    self._detection_started = True
                    self._detection_start_time = time.time()
                    
                    # Solicita detecção via service
                    if objeto_alvo and self._node._cv_detection_client.wait_for_service(timeout_sec=1.0):
                        detection_request = CVDetectionSRV.Request()
                        detection_request.object_name = objeto_alvo
                        detection_request.anomaly_types = tipos_anomalia
                        detection_request.timeout_seconds = 2.0
                        
                        future = self._node._cv_detection_client.call_async(detection_request)
                        future.add_done_callback(self._node._detection_response_callback)
                        self._node.get_logger().info(f"🔍 Solicitada detecção de '{objeto_alvo}'")
                    return
                
                # === FASE 2: Aguardar resposta da detecção ===
                elapsed = time.time() - self._detection_start_time
                if elapsed < 10.0 and self._node._detection_bbox_center is None:  # Aguarda até 10s para resposta
                    self._node.get_logger().info(f"Aguardando detecção... {elapsed:.1f}s", throttle_duration_sec=1)
                    return
                
                # === FASE 3: Verificar resultado e decidir próximo passo ===
                if self._node._detection_bbox_center is not None:
                    # Detecção bem sucedida - transicionar para APROXIMANDO
                    self._node.get_logger().info(
                        f"✅ Objeto detectado! bbox_center={self._node._detection_bbox_center}. "
                        f"Transitando para APROXIMANDO..."
                    )
                    self._node._approach_arrival_time = 0.0  # Resetar para nova aproximação
                    self._detection_started = False  # Resetar flag
                    self.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO)
                else:
                    # Detecção falhou - avança para próximo ponto
                    self._node.get_logger().warn(
                        f"❌ Objeto '{objeto_alvo}' não detectado. Avançando para próximo ponto..."
                    )
                    # Para gravação
                    if self._node._cv_record_client.wait_for_service(timeout_sec=1.0):
                        record_request = RecordDetectionsSRV.Request()
                        record_request.start_recording = False
                        self._node._cv_record_client.call_async(record_request)
                    
                    self.ponto_de_inspecao_indice_atual += 1
                    self.ponto_de_inspecao_tempo_de_chegada = 0.0
                    self._detection_started = False  # Resetar flag para próximo ponto
                    self._node._detection_bbox_center = None
                    self.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEANDO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO:
                # Escaneia por 5 segundos com detecção de anomalias habilitada
                
                # === FASE 1: Iniciar escaneamento (uma vez ao entrar no estado) ===
                if not self._scanning_started:
                    self._scanning_started = True
                    self._scanning_start_time = time.time()
                    self._node.get_logger().info("🔍 Iniciando escaneamento (5 segundos)...")
                    
                    # Inicia gravação via service
                    if self._node._cv_record_client.wait_for_service(timeout_sec=1.0):
                        record_request = RecordDetectionsSRV.Request()
                        record_request.start_recording = True
                        self._node._cv_record_client.call_async(record_request)
                        self._node.get_logger().info("🔴 Solicitado: iniciar gravação de vídeo")
                    
                    # Habilita detecção de anomalias via service
                    if self._node._cv_anomaly_detection_client.wait_for_service(timeout_sec=1.0):
                        request = EnableAnomalyDetectionSRV.Request()
                        request.enable = True
                        self._node._cv_anomaly_detection_client.call_async(request)
                        self._node.get_logger().info("🔴 Solicitado: habilitar detecção de anomalias")
                    return
                
                # === FASE 2: Aguarda 5 segundos ===
                elapsed = time.time() - self._scanning_start_time
                if elapsed < 5.0:
                    self._node.get_logger().info(
                        f"🔍 Escaneando... {5.0 - elapsed:.1f}s restantes", 
                        throttle_duration_sec=1
                    )
                    return
                
                # === FASE 3: 5 segundos passaram - transicionar para ESCANEAMENTO_FINALIZADO ===
                self._node.get_logger().info("✅ Escaneamento concluído. Transitando para ESCANEAMENTO_FINALIZADO...")
                self._scanning_started = False  # Resetar flag
                self._scanning_start_time = 0.0  # Resetar timestamp
                self.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO)

            # =================================================
            # ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO
            # =================================================
            case FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO:
                # Para gravação, desabilita detecção de anomalias e avança para próximo ponto
                
                self._node.get_logger().info("✅ Finalizando escaneamento. Parando gravação e salvando vídeo/fotos...")
                
                # Para gravação via service do CV_node
                if self._node._cv_record_client.wait_for_service(timeout_sec=1.0):
                    record_request = RecordDetectionsSRV.Request()
                    record_request.start_recording = False
                    self._node._cv_record_client.call_async(record_request)
                    self._node.get_logger().info("� Solicitado: parar gravação e salvar vídeo")
                
                # Desabilita detecção de anomalias via service
                if self._node._cv_anomaly_detection_client.wait_for_service(timeout_sec=1.0):
                    request = EnableAnomalyDetectionSRV.Request()
                    request.enable = False
                    self._node._cv_anomaly_detection_client.call_async(request)
                    self._node.get_logger().info("⬛ Solicitado: desabilitar detecção de anomalias")
                
                # Limpar variáveis e avançar para próximo ponto
                self._node._detection_bbox_center = None
                self.ponto_de_inspecao_indice_atual += 1
                self.ponto_de_inspecao_tempo_de_chegada = 0.0
                
                self._node.get_logger().info(
                    f"📍 Ponto {self.ponto_de_inspecao_indice_atual}/{len(self.mission['pontos_de_inspecao'])} concluído. "
                    f"Voltando para EXECUTANDO_INSPECIONANDO..."
                )
                self.muda_estado(FSMStateDescription.EXECUTANDO_INSPECIONANDO)

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

                # Verifica se está em trânsito RTL
                if self._node._action_in_progress or drone_state in DRONE_STATES_RTL:
                    self._node.get_logger().info(f"RTL em progresso... Estado: {drone_state.name}", throttle_duration_sec=200)
                    return
                
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
                
                # Verifica se está pronto para voar (precisa enviar RTL)
                if drone_state == DroneStateDescription.VOANDO_PRONTO:
                    self._node.get_logger().info("Enviando comando RTL...")
                    self._node.send_drone_action({"command": "RTL"})
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

    # Prefixo para destacar no terminal logs relacionados ao Drone Node
    # - Comandos enviados para Drone Node: azul + negrito
    DRONE_TX_PREFIX = "\033[34m\033[1mCOMANDO PARA DRONE NODE ENVIADO"

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
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO,
            FSMStateDescription.EXECUTANDO_INSPECIONANDO_FALHA,
            FSMStateDescription.INSPECAO_FINALIZADA,
        ],
    }
    
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
        
        # ==================================================================
        # ACTION CLIENT (comunicação com drone_node)
        # ==================================================================
        # Substitui o publisher de tópico por ActionClient para:
        # - Feedback durante execução de comandos
        # - Resultado final da ação
        # - Capacidade de cancelar ações em andamento
        self._action_client = ActionClient(
            self,
            DroneCommand,
            '/drone_inspetor/action/drone_command'
        )
        
        # Variáveis para controle de action em andamento
        self._current_goal_handle: ClientGoalHandle = None
        self._last_action_feedback = None
        self._last_action_result = None
        self._action_in_progress = False
        self._action_start_time = 0.0
        self._last_action_feedback_time = 0.0  # Tempo do último feedback recebido
        self._action_timeout = 60.0  # Timeout padrão de 60 segundos desde último feedback

        # ==================================================================
        # SERVICE CLIENTS (CV_NODE)
        # ==================================================================
        # Client para service de detecção de objetos
        self._cv_detection_client = self.create_client(
            CVDetectionSRV,
            '/drone_inspetor/interno/cv_node/srv/detection'
        )
        self.get_logger().info("Service client criado: /drone_inspetor/interno/cv_node/srv/detection")
        
        # Client para service de gravação de vídeo
        self._cv_record_client = self.create_client(
            RecordDetectionsSRV,
            '/drone_inspetor/interno/cv_node/srv/record_detections'
        )
        self.get_logger().info("Service client criado: /drone_inspetor/interno/cv_node/srv/record_detections")
        
        # Client para service de habilitação de detecção de anomalias
        self._cv_anomaly_detection_client = self.create_client(
            EnableAnomalyDetectionSRV,
            '/drone_inspetor/interno/cv_node/srv/enable_anomaly_detection'
        )
        self.get_logger().info("Service client criado: /drone_inspetor/interno/cv_node/srv/enable_anomaly_detection")
        
        # Variáveis para controle de aproximação (usadas em estados APROXIMANDO/ESCANEANDO)
        self._detection_bbox_center = None    # Centro do bbox detectado [x, y]
        self._approach_arrival_time = 0.0     # Tempo de chegada na aproximação
        self._previous_inspection_position = None  # (lat, lon, alt) antes de aproximar

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
        # Timer principal da FSM - executa a atualização e publicação do estado da FSM a cada 600ms
        self.fsm_timer = self.create_timer(0.6, self.update_and_publish_fsm_state)
        
        # Timer para verificação de saúde dos tópicos essenciais - executa a cada 2 segundos
        self.topic_health_timer = self.create_timer(2.0, self.check_essential_topics)
        
        self.get_logger().info("================ FSM NODE PRONTO ================")

    # ==================================================================
    # MÉTODOS AUXILIARES DE CONFIGURAÇÃO
    # ==================================================================
    
    def get_missions_directory(self) -> str:
        """
        Obtém o diretório base para salvar dados das missões.
        Lê do parâmetro global do config params.yaml.
        
        Returns:
            Caminho do diretório de missões (padrão: ~/Drone_Inspetor_Missoes)
        """
        # TODO: Implementar leitura do params.yaml via rosparam ou arquivo direto
        # Por enquanto retorna o valor padrão
        return "~/Drone_Inspetor_Missoes"

    # ==================================================================
    # TIMERS - Funções Principais (executadas periodicamente)
    # ==================================================================

    def update_and_publish_fsm_state(self):
        """
        Atualiza e publica o estado da FSM a cada 0.6s.
        
        """
        self.fsm_state.verifica_mudanca_de_estado()
        self.fsm_state.publish()


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
            self.send_drone_action({"command": "STOP"})
            
            # Reseta a FSM
            self.fsm_state.reset()
            
            return False
        
        return True

    def send_drone_action(self, command_dict):
        """
        Envia um comando para o drone_node usando DroneCommand Action.
        
        Args:
            command_dict: Dicionário com o comando a ser enviado.
            
        Returns:
            bool: True se o goal foi enviado com sucesso, False caso contrário.
        """
        # Verifica se já existe uma ação em progresso
        if self._action_in_progress:
            cmd = command_dict.get("command", "")
            if cmd == "STOP":
                self.get_logger().warn("Enviando comando STOP mesmo com ação em progresso (Prioridade de Emergência)")
                self.cancel_current_action()
            else:
                self.get_logger().warn(f"Ignorando comando {cmd}: Já existe uma ação em progresso!")
                return False
        
        # Marca a ação como em progresso
        self._action_in_progress = True
        self._action_start_time = time.time()
        
        # Verifica se o action server está disponível
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server /drone_inspetor/action/drone_command não disponível!")
            self._action_in_progress = False
            self._action_start_time = 0.0
            return False
            
        # Cria o goal
        goal_msg = DroneCommand.Goal()
        goal_msg.command = command_dict.get("command", "")
        
        # Parâmetros para GOTO - define NaN se não fornecido (indica "não especificado")
        goal_msg.lat = command_dict.get("lat", float('nan'))
        goal_msg.lon = command_dict.get("lon", float('nan'))
        goal_msg.alt = command_dict.get("alt", float('nan'))
        goal_msg.yaw = command_dict.get("yaw", float('nan'))
        
        # Parâmetros para GOTO_FOCUS
        goal_msg.focus_lat = command_dict.get("focus_lat", float('nan'))
        goal_msg.focus_lon = command_dict.get("focus_lon", float('nan'))
        
        # Parâmetros para TAKEOFF
        if "altitude" in command_dict:
            goal_msg.altitude = command_dict["altitude"]
        elif "alt" in command_dict:
            goal_msg.altitude = command_dict["alt"]
        else:
            goal_msg.altitude = float('nan')

        self.get_logger().info(f"{self.DRONE_TX_PREFIX} - {goal_msg.command}")
        
        # Envia o goal de forma assíncrona
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._action_feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True

    def _goal_response_callback(self, future):
        """
        Callback chamado quando o goal é aceito ou rejeitado.
        
        Args:
            future: Future com o resultado da requisição de goal
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejeitado pelo drone_node!")
            self._action_in_progress = False
            self._action_start_time = 0.0
            return
        
        self.get_logger().info("Goal aceito pelo drone_node, aguardando execução...")
        self._current_goal_handle = goal_handle
        
        # Configura callback para quando o resultado estiver disponível
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._action_result_callback)

    def _action_feedback_callback(self, feedback_msg):
        """
        Callback chamado quando feedback é recebido durante execução da action.
        Atualiza o timer de feedback para manter a action ativa enquanto houver progresso.
        
        Args:
            feedback_msg: Mensagem de feedback com progresso
        """
        feedback = feedback_msg.feedback
        self._last_action_feedback = feedback
        
        # Atualiza o tempo do último feedback (usado para timeout)
        self._last_action_feedback_time = time.time()
        
        self.get_logger().debug(
            f"Feedback Action: estado = {feedback.state_name}, "
            f"distância = {feedback.distance_to_target:.2f}m, "
            f"progresso = {feedback.progress_percent:.1f}%",
            throttle_duration_sec=1.0
        )

    def _action_result_callback(self, future):
        """
        Callback chamado quando a action é completada.
        
        Args:
            future: Future com o resultado da action
        """
        result = future.result().result
        self._last_action_result = result
        self._action_in_progress = False
        self._action_start_time = 0.0
        self._last_action_feedback_time = 0.0
        self._current_goal_handle = None
        
        if result.success:
            self.get_logger().info(f"Action completada com sucesso: {result.message}")
            
            # Se o comando foi bem sucedido e estamos inspecionando, assumimos chegada ao waypoint
            if self.fsm_state.state == FSMStateDescription.EXECUTANDO_INSPECIONANDO:
                self.get_logger().info("Waypoint alcançado (Action Success)! Iniciando contagem de tempo.")
                self.fsm_state.ponto_de_inspecao_tempo_de_chegada = time.time()
                
        else:
            self.get_logger().warn(f"Action falhou: {result.message}")

    def cancel_current_action(self):
        """
        Cancela a action em andamento, se houver.
        
        Returns:
            bool: True se havia uma action para cancelar, False caso contrário.
        """
        if self._current_goal_handle is not None:
            self.get_logger().info("Cancelando action em andamento...")
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            return True
        return False

    def _cancel_done_callback(self, future):
        """
        Callback chamado quando o cancelamento é processado.
        
        Args:
            future: Future com o resultado do cancelamento
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Action cancelada com sucesso")
        else:
            self.get_logger().warn("Falha ao cancelar action")
    
    def _detection_response_callback(self, future):
        """
        Callback chamado quando a resposta do service de detecção CV é recebida.
        
        Args:
            future: Future com o resultado do service CVDetectionSRV
        """
        try:
            response = future.result()
            
            if response.success and len(response.bbox_center) >= 2:
                # Usa o centro do bounding box retornado pelo service
                bbox_center_x = response.bbox_center[0]
                bbox_center_y = response.bbox_center[1]
                
                self._detection_bbox_center = (bbox_center_x, bbox_center_y)
                
                self.get_logger().info(
                    f"🎯 Detecção bem sucedida! {response.message}, "
                    f"Confiança: {response.confidence:.2f}, "
                    f"BBox Center: ({bbox_center_x:.1f}, {bbox_center_y:.1f})"
                )
            else:
                self._detection_bbox_center = None
                self.get_logger().warn(
                    f"❌ {response.message if response.message else 'Objeto não detectado'}"
                )
                
        except Exception as e:
            self._detection_bbox_center = None
            self.get_logger().error(f"Erro ao processar resposta de detecção: {e}")

    def check_action_timeout(self) -> bool:
        """
        Verifica se a action em andamento excedeu o timeout desde o último feedback.
        Se excedeu, cancela a action e retorna True.
        
        Returns:
            bool: True se houve timeout e a action foi cancelada, False caso contrário.
        """
        if not self._action_in_progress:
            return False
        
        # Usa o tempo do último feedback para calcular o timeout
        # Se ainda não recebeu feedback, usa o tempo de início da action
        reference_time = self._last_action_feedback_time if self._last_action_feedback_time > 0 else self._action_start_time
        elapsed = time.time() - reference_time
        
        if elapsed > self._action_timeout:
            self.get_logger().error(
                f"TIMEOUT na action! ({elapsed:.1f}s desde último feedback > {self._action_timeout:.1f}s). Cancelando..."
            )
            self.cancel_current_action()
            return True
        
        return False

    def verifica_validade_do_comando(self, command_enum: DashboardFsmCommandDescription) -> tuple[bool, str]:
        """
        Valida se um comando do dashboard é permitido no estado atual.
        
        Args:
            command_enum: Enum do comando já convertido
            
        Returns:
            tuple: (can_execute, error_message)
                - can_execute: True se comando é permitido, False se não
                - error_message: Mensagem de erro se não permitido, string vazia se permitido
        """
        allowed_states = self.VALID_DASHBOARD_COMMANDS.get(command_enum, [])
        
        if not allowed_states:
            return False, f"Comando '{command_enum.name}' não está configurado em VALID_DASHBOARD_COMMANDS"
        
        current_state = self.fsm_state.state
        if current_state in allowed_states:
            return True, ""
        else:
            allowed_names = [s.name for s in allowed_states]
            return False, (
                f"Comando '{command_enum.name}' não permitido no estado {current_state.name}. "
                f"Estados permitidos: {allowed_names}"
            )


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
        can_execute, error_msg = self.verifica_validade_do_comando(command_enum)
        
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
        # TODO: Implementar lógica de detecção de CV para inspeção
        pass



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
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
