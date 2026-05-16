# =================================================================================================
# DroneFSMContext — variáveis da DroneFSM (lifecycle do drone)
# =================================================================================================
# Pareado com DroneFSM. Contém EXCLUSIVAMENTE as variáveis usadas pelos estados de
# lifecycle (OFFBOARD_DESATIVADO, POUSADO_DESARMADO, POUSADO_ARMADO, DECOLANDO, EM_VOO,
# EMERGENCIA).
#
# NÃO contém estado da TargetStack, last_static_position, tolerâncias de manobra —
# tudo isso pertence ao DeslocamentoFSMContext.
#
# Acesso a recursos compartilhados (`state_px4`, `obstacles`) é feito via `self.node.<recurso>`,
# já que esses subsistemas pertencem ao DroneNode e não a uma FSM específica.
# =================================================================================================

from typing import TYPE_CHECKING

from px4_msgs.msg import VehicleStatus

from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode


# =================================================================================================
# Mapeamento comando → lifecycle states permitidos
# =================================================================================================
VALID_COMMANDS = {
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
        # Valores: None | "ARM" | "TAKEOFF" | "GOTO" | "LAND" | "RTL".
        self.pending_command: 'str | None' = None
        # Modo de GOTO pendente: True → "manter yaw apontando ao foco".
        self.pending_use_focus: bool = False

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
        if battery and battery.remaining < 0.10:
            self.node.get_logger().warn("EMERGÊNCIA: Bateria crítica!")
            self.emergency_active = True
            return True
        return False

    def verifica_validade_do_comando(self, command: str) -> tuple[bool, str]:
        """
        Verifica se um comando pode ser executado no lifecycle atual.

        Pré-condições gerais:
            - PX4 está em modo OFFBOARD.
            - Posição local conhecida.
        Pré-condição específica:
            - lifecycle atual permite este comando (ver VALID_COMMANDS).

        Returns:
            (pode_executar, mensagem_de_erro). Erro vazio se OK.
        """
        px4 = self.node.state_px4
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return False, "Drone NÃO está em modo OFFBOARD."
        if px4.local_position is None:
            return False, "Posição local desconhecida."

        allowed = VALID_COMMANDS.get(command, [])
        if not allowed:
            return True, ""
        if self.state in allowed:
            return True, ""

        allowed_names = [s.name for s in allowed]
        return False, (
            f"Comando '{command}' não permitido no estado {self.state.name}. "
            f"Estados permitidos: {allowed_names}"
        )
