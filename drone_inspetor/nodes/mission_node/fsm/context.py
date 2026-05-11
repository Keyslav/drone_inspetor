# context.py
# =================================================================================================
# CONTEXTO COMPARTILHADO DA FSM DE MISSÃO
# =================================================================================================
# Contém dados da missão (waypoints, flags, parâmetros), métodos de publicação,
# validação de missão e reset. Cada State recebe uma referência a este contexto.
# =================================================================================================

import json
import os
from datetime import datetime
from typing import TYPE_CHECKING

from drone_inspetor_msgs.msg import MissionStateMSG
from drone_inspetor_msgs.srv import CVDetectionSRV, RecordDetectionsSRV, EnableAnomalyDetectionSRV

from drone_inspetor.common.enums import MissionStateDescription
from drone_inspetor.common.log_colors import Ansi, colorir

if TYPE_CHECKING:
    from drone_inspetor.nodes.mission_node.mission_node import MissionNode


class MissionFSMContext:
    """
    Dados compartilhados entre todos os estados da FSM de missão.
    Encapsula configuração de missão, waypoints, flags de controle e publicação.
    """

    def __init__(self, node: 'MissionNode'):
        self.node = node

        # Estado atual (espelhado da StateMachine para acesso rápido)
        self.state = MissionStateDescription.DESATIVADO

        # Flags de missão
        self.on_mission = False
        self.cancel_mission = False

        # Missões disponíveis e missão atual
        self.missions = {}
        self.mission = None
        self._load_missions()

        # Parâmetros de configuração
        self.takeoff_altitude = 20.0
        self.tempo_de_permanencia = 5.0

        # Controle de waypoints
        self.ponto_de_inspecao_indice_atual = 0
        self.ponto_de_inspecao_tempo_de_chegada = 0.0
        self._detection_started = False
        self._detection_start_time = 0.0
        self._scanning_started = False
        self._scanning_start_time = 0.0

        # Diretório da missão atual
        self.mission_folder_path = ""

    # ------------------------------------------------------------------
    # Utilitários
    # ------------------------------------------------------------------

    def now(self) -> float:
        """Timestamp atual em segundos."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def get_ponto_atual(self) -> dict | None:
        """Retorna o dicionário do ponto de inspeção atual, ou None."""
        if not self.mission or 'pontos_de_inspecao' not in self.mission:
            return None
        pontos = self.mission['pontos_de_inspecao']
        if 0 <= self.ponto_de_inspecao_indice_atual < len(pontos):
            return pontos[self.ponto_de_inspecao_indice_atual]
        return None

    def total_pontos(self) -> int:
        """Número total de pontos de inspeção na missão atual."""
        if self.mission and 'pontos_de_inspecao' in self.mission:
            return len(self.mission['pontos_de_inspecao'])
        return 0

    # ------------------------------------------------------------------
    # Carregamento e validação de missão
    # ------------------------------------------------------------------

    def _load_missions(self):
        """Carrega missões do arquivo missions.json."""
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('drone_inspetor')
        missions_file = os.path.join(package_share_dir, 'missions', 'missions.json')
        try:
            with open(missions_file, 'r') as f:
                self.missions = json.load(f)
            self.node.get_logger().info(f"Missões carregadas: {list(self.missions.keys())}")
        except FileNotFoundError:
            self.node.get_logger().error(f"Arquivo de missões não encontrado: {missions_file}")
            self.missions = {}
        except json.JSONDecodeError as e:
            self.node.get_logger().error(f"Erro ao parsear missions.json: {e}")
            self.missions = {}

    def valida_missao(self, mission_name: str) -> tuple[bool, str]:
        """Valida se uma missão existe e pode ser carregada."""
        if not mission_name:
            return False, "Nome da missão não especificado"
        if mission_name not in self.missions:
            return False, f"Missão '{mission_name}' não encontrada. Disponíveis: {list(self.missions.keys())}"
        self.mission = self.missions[mission_name]
        self.ponto_de_inspecao_indice_atual = 0
        self.mission_folder_path = self._create_mission_folder(mission_name)
        self.node.get_logger().info(f"Missão '{mission_name}' carregada. Pasta: {self.mission_folder_path}")
        return True, ""

    def _create_mission_folder(self, mission_name: str) -> str:
        """Cria a pasta da missão com timestamp."""
        missions_directory = os.path.expanduser(self.node.get_missions_directory())
        folder_path = os.path.join(missions_directory, f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        try:
            os.makedirs(os.path.join(folder_path, "fotos"), exist_ok=True)
            os.makedirs(os.path.join(folder_path, "videos"), exist_ok=True)
            self.node.get_logger().info(f"Pasta da missão criada: {folder_path}")
        except OSError as e:
            self.node.get_logger().error(f"Erro ao criar pasta da missão: {e}")
            return ""
        return folder_path

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(self):
        """Reseta variáveis para nova missão. Cancela action em andamento."""
        self.node.cancel_current_action()
        self.state = MissionStateDescription.DESATIVADO
        self.on_mission = False
        self.cancel_mission = False
        self.mission = None
        self.mission_folder_path = ""
        self.ponto_de_inspecao_indice_atual = 0
        self.ponto_de_inspecao_tempo_de_chegada = 0.0
        self._detection_started = False
        self._scanning_started = False
        self.node.get_logger().info(colorir("Máquina de Estados de Missão resetada!", Ansi.NEGRITO, Ansi.AMARELO_CLARO))

    # ------------------------------------------------------------------
    # Publicação
    # ------------------------------------------------------------------

    def publish(self):
        """Publica estado atual no tópico ROS."""
        msg = MissionStateMSG()
        msg.state = int(self.state)
        msg.state_name = self.state.name
        msg.on_mission = self.on_mission
        msg.cancel_mission = self.cancel_mission
        msg.mission_name = self.mission.get('nome', '') if self.mission else ""
        msg.mission_folder_path = self.mission_folder_path
        msg.tempo_de_permanencia = self.tempo_de_permanencia
        msg.takeoff_altitude = self.takeoff_altitude
        msg.ponto_de_inspecao_indice_atual = self.ponto_de_inspecao_indice_atual
        msg.total_pontos_de_inspecao = self.total_pontos()
        msg.ponto_de_inspecao_tempo_de_chegada = self.ponto_de_inspecao_tempo_de_chegada

        ponto = self.get_ponto_atual()
        msg.objeto_alvo = ponto.get('objeto_alvo', '') if ponto else ""
        msg.tipos_anomalia = ponto.get('tipos_anomalia', []) if ponto else []

        self.node.mission_state_pub.publish(msg)
