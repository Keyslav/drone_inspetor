# waypoint_stack.py
# =================================================================================================
# PILHA DE COORDENADAS DE NAVEGAÇÃO
# =================================================================================================
# Gerencia uma pilha LIFO de waypoints de missão (GOTO/RTL). O sistema de
# desvio de obstáculo NÃO usa esta pilha — vive na sub-FSM *_OBSTACULO_*
# e em campos paralelos do DroneFSMContext.
# =================================================================================================

from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional


class WaypointType(Enum):
    """Tipo do waypoint na pilha."""
    MISSAO = auto()       # Waypoint original da missão (GOTO/RTL)


class WaypointStatus(Enum):
    """Status de um waypoint na pilha."""
    PENDENTE = auto()     # Aguardando execução
    ATIVO = auto()        # Em execução (drone voando para ele)
    CONCLUIDO = auto()    # Drone chegou nesse ponto


@dataclass
class Waypoint:
    """
    Um ponto de navegação na pilha.

    Atributos obrigatórios:
        local_position: [x, y, z] em coordenadas locais NED
        waypoint_type: MISSAO

    Atributos opcionais (preenchidos para waypoints de missão):
        latitude, longitude, altitude: coordenadas globais para log/feedback
        direction_yaw_deg: yaw de direção para apontar antes de voar (0-360)
        direction_yaw_deg_normalized: yaw de direção normalizado (-180/180)
        direction_yaw_rad: yaw de direção em radianos
        final_yaw_deg: yaw final ao chegar no destino (0-360)
        final_yaw_deg_normalized: yaw final normalizado (-180/180)
        final_yaw_rad: yaw final em radianos
        focus_local_position: [x, y, z] do ponto de foco (GOTO com use_focus=True)
        focus_latitude, focus_longitude: coordenadas globais do foco
    """
    local_position: list
    waypoint_type: WaypointType
    status: WaypointStatus = WaypointStatus.PENDENTE

    # Coordenadas globais opcionais (para log/feedback)
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None

    # Yaw de direção (calculado ao empilhar)
    direction_yaw_deg: Optional[float] = None
    direction_yaw_deg_normalized: Optional[float] = None
    direction_yaw_rad: Optional[float] = None

    # Yaw final (ao chegar no destino)
    final_yaw_deg: Optional[float] = None
    final_yaw_deg_normalized: Optional[float] = None
    final_yaw_rad: Optional[float] = None

    # Metadados de foco (GOTO com use_focus=True)
    focus_local_position: Optional[list] = None
    focus_latitude: Optional[float] = None
    focus_longitude: Optional[float] = None


class WaypointStack:
    """
    Pilha LIFO de waypoints de missão.

    Hoje é usada apenas para empilhar destinos de GOTO/RTL. A lógica de
    desvio de obstáculo passou a ser tratada pela sub-FSM *_OBSTACULO_*
    (ver drone_node/fsm/states/_obstaculo_base.py), que mantém suas próprias
    pilhas no DroneFSMContext.
    """

    def __init__(self):
        self._stack: list[Waypoint] = []

    # ------------------------------------------------------------------
    # Propriedades
    # ------------------------------------------------------------------

    @property
    def current(self) -> Optional[Waypoint]:
        """Retorna o waypoint ativo ou pendente mais recente (topo da pilha)."""
        for wp in reversed(self._stack):
            if wp.status in (WaypointStatus.ATIVO, WaypointStatus.PENDENTE):
                return wp
        return None

    @property
    def is_empty(self) -> bool:
        """True se não há waypoints ativos ou pendentes."""
        return self.current is None

    @property
    def active_count(self) -> int:
        """Número de waypoints ainda pendentes ou ativos."""
        return sum(
            1 for wp in self._stack
            if wp.status in (WaypointStatus.ATIVO, WaypointStatus.PENDENTE)
        )

    @property
    def total_count(self) -> int:
        """Número total de waypoints na pilha (incluindo concluídos)."""
        return len(self._stack)

    # ------------------------------------------------------------------
    # Operações de pilha
    # ------------------------------------------------------------------

    def push_mission(self, local_pos: list, **kwargs) -> Waypoint:
        """
        Empilha um waypoint de missão (destino final de GOTO/RTL).

        Args:
            local_pos: [x, y, z] posição local NED
            **kwargs: Atributos opcionais do Waypoint (latitude, longitude, etc.)

        Returns:
            O Waypoint criado
        """
        wp = Waypoint(
            local_position=list(local_pos),
            waypoint_type=WaypointType.MISSAO,
            status=WaypointStatus.PENDENTE,
            **kwargs,
        )
        self._stack.append(wp)
        return wp

    def complete_current(self) -> Optional[Waypoint]:
        """
        Marca o waypoint ativo como CONCLUIDO e reativa o próximo abaixo dele.

        Percorre a pilha de cima para baixo procurando o primeiro waypoint
        PENDENTE para reativar.

        Returns:
            O próximo waypoint que se tornou ativo, ou None se não há mais.
        """
        current = self.current
        if current is not None:
            current.status = WaypointStatus.CONCLUIDO

        # Reativa o próximo waypoint PENDENTE (de cima para baixo)
        for wp in reversed(self._stack):
            if wp.status == WaypointStatus.PENDENTE:
                wp.status = WaypointStatus.ATIVO
                return wp
        return None

    def clear(self) -> None:
        """Limpa toda a pilha (usado por STOP e reset)."""
        self._stack.clear()

    # ------------------------------------------------------------------
    # Representação
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        items = []
        for i, wp in enumerate(self._stack):
            pos = wp.local_position
            items.append(
                f"  [{i}] {wp.waypoint_type.name}({wp.status.name}) "
                f"pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
            )
        return f"WaypointStack({self.active_count} ativos, {self.total_count} total):\n" + "\n".join(items)
