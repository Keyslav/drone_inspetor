# =================================================================================================
# OffboardDesativadoState — estado da DroneFSM
# =================================================================================================
# DroneFSMDescription.OFFBOARD_DESATIVADO
#
# Significado: o PX4 está fora do modo OFFBOARD (operando em MANUAL, POSCTL, AUTO_*, etc.),
# então o DroneNode não pode enviar setpoints de trajetória ao autopilot.
#
# Transições:
#     PX4 entra em OFFBOARD + drone está pousado e desarmado → POUSADO_DESARMADO
#     PX4 entra em OFFBOARD + drone está pousado e armado   → POUSADO_ARMADO
#     PX4 entra em OFFBOARD + drone está no ar              → EM_VOO
# =================================================================================================

from px4_msgs.msg import VehicleStatus

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class OffboardDesativadoState(BaseState):
    """Drone aguardando o PX4 entrar em modo OFFBOARD."""

    def on_enter(self) -> None:
        # Garante que nenhuma trajetória residual esteja ativa ao entrar.
        self.node.deslocamento_fsm_context.reset()
        self.node.get_logger().info(
            "Drone fora do modo OFFBOARD. Aguardando ativação do modo offboard no PX4.",
            throttle_duration_sec=5.0,
        )

    def on_step(self):
        px4 = self.node.state_px4

        # Aguardamos OFFBOARD: sem isso, nada a fazer.
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return None

        # Modo OFFBOARD ativo: rotear para o estado correspondente à situação física do drone.
        if not px4.is_armed:
            return DS.POUSADO_DESARMADO
        if px4.is_landed:
            return DS.POUSADO_ARMADO
        # Armado e no ar — caso raro (ex.: usuário alternou modos durante voo manual).
        return DS.EM_VOO
