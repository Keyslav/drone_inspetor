# =================================================================================================
# DeslocamentoFSMContext — variáveis da DeslocamentoFSM (fases de manobra)
# =================================================================================================
# Pareado com DeslocamentoFSM. Contém EXCLUSIVAMENTE as variáveis usadas pelos estados
# de manobra (PLANANDO, GIRANDO_INICIO, DESLOCANDO, GIRANDO_FIM):
#
#     - TargetStack    → pilha de destinos (MISSAO/DESVIO).
#     - last_static_*  → referência de hover.
#     - tolerâncias    → critérios de chegada e estabilização.
#     - timers         → yaw_aligned_time, trajectory_start_time.
#
# Acesso a recursos compartilhados (`state_px4`, `obstacles`) é feito via `self.node.<recurso>`.
# =================================================================================================

from typing import TYPE_CHECKING

from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription
from drone_inspetor.nodes.drone_node.target_stack import TargetStack

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode


class DeslocamentoFSMContext:
    """Contexto da DeslocamentoFSM (manobra). Encapsula apenas o que ela lê/escreve."""

    def __init__(self, node: 'DroneNode'):
        # ---- Referência ao nó hospedeiro (logger, clock, state_px4, obstacles). ----
        self.node = node

        # ---- Espelho do estado atual da FSM (atualizado em DeslocamentoFSM.transition_to). ----
        self.state: DeslocamentoFSMDescription = DeslocamentoFSMDescription.PLANANDO

        # ---- Pilha de alvos (MISSAO + DESVIOs encadeados). ----
        self.target_stack = TargetStack()

        # ---- Referência de hover (atualizada pelos estados ao "parar" o drone). ----
        self.last_static_position: 'list | None' = None
        self.last_static_yaw_deg: 'float | None' = None
        self.last_static_yaw_deg_normalized: 'float | None' = None
        self.last_static_yaw_rad: 'float | None' = None

        # ---- Auxiliares de manobra (gestão de tempo dentro dos estados) ----
        # Timestamp do instante em que o yaw alvo foi atingido. None = ainda não atingido.
        self.yaw_aligned_time: 'float | None' = None
        # Timestamp em que a manobra ativa começou.
        self.trajectory_start_time: 'float | None' = None
        # Distância inicial ao target ativo (usada para cálculo de progresso percentual).
        self.initial_distance_to_target: 'float | None' = None

        # ---- Parâmetros de tolerância (configuráveis no futuro via param_ros.yaml). ----
        # Distância (m) para considerar "chegou no target" (norma 3D).
        self.position_tolerance: float = 0.15
        # Tolerância angular (graus) para considerar yaw alinhado.
        self.yaw_tolerance_deg: float = 2.0
        # Tempo (s) que o drone permanece parado após yaw alinhado antes de transicionar.
        self.yaw_stabilization_delay: float = 3.0
        # Passo angular incremental usado pela rotação suave (graus por tick).
        self.yaw_step_deg: float = 15.0

    # =============================================================================================
    # Utilitários temporais
    # =============================================================================================

    def now(self) -> float:
        """Timestamp atual (segundos) — relógio do nó ROS2."""
        return self.node.get_clock().now().nanoseconds / 1e9

    # =============================================================================================
    # Snapshot de hover
    # =============================================================================================

    def store_static_position(self) -> None:
        """
        Captura posição/yaw atuais como referência estática de hover.

        Chamado sempre que o drone "para" — entrada em PLANANDO, fim de manobra, etc.
        A Trajectory consulta esses campos para gerar setpoints de hover constantes.
        """
        px4 = self.node.state_px4
        if px4.local_position is None:
            return
        self.last_static_position = [
            px4.local_position.x,
            px4.local_position.y,
            px4.local_position.z,
        ]
        yaw_norm = px4.current_yaw_deg_normalized
        self.last_static_yaw_deg_normalized = yaw_norm
        self.last_static_yaw_deg = yaw_norm if yaw_norm >= 0 else yaw_norm + 360
        self.last_static_yaw_rad = px4.current_yaw_rad

    # =============================================================================================
    # Reset
    # =============================================================================================

    def reset(self) -> None:
        """
        Limpa todo o estado da manobra. Chamado em:
            - STOP, cancelamento, pouso, emergência, perda de offboard.

        Antes de limpar, captura a posição atual como hover (evita drift quando o reset
        ocorre em pleno voo).
        """
        self.store_static_position()
        self.target_stack.clear()
        self.yaw_aligned_time = None
        self.trajectory_start_time = None
        self.initial_distance_to_target = None

    # =============================================================================================
    # Helper angular (delegado a math_utils)
    # =============================================================================================

    def yaw_diff_shortest(self, current_deg: float, target_deg: float) -> float:
        """Diferença angular pelo caminho mais curto (-180 a 180)."""
        from drone_inspetor.common.math_utils import yaw_diff_shortest
        return yaw_diff_shortest(current_deg, target_deg)
