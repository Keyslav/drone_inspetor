"""Dados de uma sessão de missão, sem publicação, arquivos ou mudanças de estado."""

from dataclasses import dataclass

from drone_inspetor.missions.models import MissionDefinition, Waypoint


@dataclass
class MissionFSMContext:
    """A FSM é dona do estado; o contexto possui somente os dados da sessão."""

    mission: MissionDefinition | None = None
    mission_folder_path: str = ''
    on_mission: bool = False
    cancel_mission: bool = False
    ponto_de_inspecao_indice_atual: int = 0
    ponto_de_inspecao_tempo_de_chegada: float = 0.0
    waypoint_reached: bool = False
    failure_reason: str = ''

    @property
    def takeoff_altitude(self):
        """Altitude relativa ao home usada pela missão validada."""
        return self.mission.takeoff_altitude_m if self.mission else 20.0

    @property
    def tempo_de_permanencia(self):
        """Duração ROS da inspeção atual; permite override por waypoint."""
        point = self.get_ponto_atual()
        if point is not None and point.dwell_s is not None:
            return point.dwell_s
        return self.mission.dwell_s if self.mission else 5.0

    def get_ponto_atual(self) -> Waypoint | None:
        """Retorna o waypoint validado da sessão atual."""
        if self.mission and 0 <= self.ponto_de_inspecao_indice_atual < self.total_pontos():
            return self.mission.waypoints[self.ponto_de_inspecao_indice_atual]
        return None

    def total_pontos(self):
        """Quantidade de pontos na definição imutável."""
        return len(self.mission.waypoints) if self.mission else 0

    def start(self, definition, directory):
        """Instala uma definição e diretório já preparados pelo coordenador."""
        self.clear()
        self.mission = definition
        self.mission_folder_path = directory
        self.on_mission = True

    def advance_waypoint(self):
        """Avança sem reaproveitar a chegada do ponto anterior."""
        self.ponto_de_inspecao_indice_atual += 1
        self.ponto_de_inspecao_tempo_de_chegada = 0.0
        self.waypoint_reached = False

    def clear(self):
        """Limpa dados; não cancela transporte nem altera a máquina de estados."""
        self.mission = None
        self.mission_folder_path = ''
        self.on_mission = False
        self.cancel_mission = False
        self.ponto_de_inspecao_indice_atual = 0
        self.ponto_de_inspecao_tempo_de_chegada = 0.0
        self.waypoint_reached = False
        self.failure_reason = ''
