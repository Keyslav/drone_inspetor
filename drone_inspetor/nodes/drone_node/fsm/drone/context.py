"""Dados do ciclo de voo, admissão de comandos e intenção de transferência ao PX4."""

from typing import TYPE_CHECKING
import math

from px4_msgs.msg import VehicleStatus

from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode


# =================================================================================================
# Mapeamento comando → lifecycle states permitidos
# =================================================================================================
VALID_COMMANDS = {
    "DISARM": [DroneFSMDescription.POUSADO_ARMADO],
    "STOP": [DroneFSMDescription.EM_VOO, DroneFSMDescription.DECOLANDO],
    "ARM":     [DroneFSMDescription.POUSADO_DESARMADO],
    "TAKEOFF": [DroneFSMDescription.POUSADO_ARMADO],
    "GOTO":    [DroneFSMDescription.EM_VOO],
    "LAND":    [DroneFSMDescription.EM_VOO],
    "RTL":     [DroneFSMDescription.EM_VOO],
}


class DroneFSMContext:
    """Contexto da DroneFSM (lifecycle). Encapsula apenas o que a DroneFSM lê/escreve."""

    def __init__(self, node: 'DroneNode'):
        # ---- Referência ao nó hospedeiro (logger, clock, state_px4, obstacles). ----
        self.node = node

        # ---- Espelho do estado atual da FSM (atualizado em DroneFSM.transition_to). ----
        # Inicializa em OFFBOARD_DESATIVADO; a primeira transição efetiva é feita no startup.
        self.state: DroneFSMDescription = DroneFSMDescription.OFFBOARD_DESATIVADO
        # Timestamp em segundos da última transição. Usado para medir tempo de permanência.
        self.state_entry_time: float = self.now()

        # ---- Comando pendente recebido via Action (consumido pelo estado correspondente). ----
        # ARM/TAKEOFF são consumidos pelos estados de solo; GOTO é preparado de imediato.
        self.pending_command: 'str | None' = None
        # Modo de GOTO pendente: True → "manter yaw apontando ao foco".
        self.pending_use_focus: bool = False
        self.native_command: str | None = None
        self.native_mode_observed = False

        # ---- Parâmetros lifecycle ----
        # Altitude alvo de decolagem (m). Sobrescrita pelo Action TAKEOFF.
        self.takeoff_altitude: float = 2.5

        # ---- Flag de failsafe ativo (controle delegado ao autopilot). ----
        self.emergency_active: bool = False

    # =============================================================================================
    # Utilitários temporais
    # =============================================================================================

    def now(self) -> float:
        """Timestamp atual (segundos) — relógio do nó ROS2."""
        return self.node.get_clock().now().nanoseconds / 1e9

    # =============================================================================================
    # Verificações globais (usadas pela DroneFSM.tick antes de delegar ao estado atual)
    # =============================================================================================

    def verifica_condicao_de_emergencia(self) -> bool:
        """
        Detecta emergência (bateria crítica).

        Returns:
            True apenas na PRIMEIRA detecção. Subsequente é suprimida por `emergency_active`
            para evitar reenvio do RTL emergencial.
        """
        if self.emergency_active:
            return False
        battery = self.node.state_px4.battery_status
        if (self.node.state_px4.is_armed and battery
                and math.isfinite(battery.remaining) and 0 <= battery.remaining < 0.10):
            self.node.get_logger().warn("EMERGÊNCIA: Bateria crítica!")
            self.emergency_active = True
            return True
        return False

    def verifica_validade_do_comando(self, command: str) -> tuple[bool, str]:
        """
        Verifica se um comando pode ser executado no lifecycle atual.

        Pré-condições gerais:
            - PX4 está em modo OFFBOARD.
            - Posição local conhecida, recente e referência de frenagem concluída.
            - A FSM confirma estabilização medida antes de iniciar movimento.
            - Controle não está delegado a um comando nativo.
        Pré-condição específica:
            - lifecycle atual permite este comando (ver VALID_COMMANDS).

        Returns:
            (pode_executar, mensagem_de_erro). Erro vazio se OK.
        """
        if command not in VALID_COMMANDS:
            return False, f"Comando desconhecido: {command}"
        px4 = self.node.state_px4
        if self.native_command is not None:
            return False, "Controle delegado ao PX4; aguardando conclusão nativa."
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return False, "Drone NÃO está em modo OFFBOARD."
        if px4.local_position is None:
            return False, "Posição local desconhecida."

        if not self.node.telemetry_fresh():
            return False, "Telemetria local expirada."
        if command in ('ARM', 'TAKEOFF', 'DISARM') and not px4.is_landed:
            return False, f'{command} é permitido somente no solo.'
        if command in ('ARM', 'TAKEOFF', 'GOTO') and not self.node.trajectory.reference_stopped:
            return False, "A referência da frenagem anterior ainda está em movimento."
        allowed = VALID_COMMANDS[command]
        if self.state in allowed:
            return True, ""

        allowed_names = [s.name for s in allowed]
        return False, (
            f"Comando '{command}' não permitido no estado {self.state.name}. "
            f"Estados permitidos: {allowed_names}"
        )

    def accepts_native_mode(self):
        """Distingue transferência esperada de controle de mudança manual/falha."""
        state = self.node.state_px4.nav_state
        accepted = {
            'LAND': {VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
                     VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND},
            'RTL': {VehicleStatus.NAVIGATION_STATE_AUTO_RTL,
                    VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
                    VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND},
        }.get(self.native_command, set())
        if state in accepted:
            self.native_mode_observed = True
            return True
        return False
