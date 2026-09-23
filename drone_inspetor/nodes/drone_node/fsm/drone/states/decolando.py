# =================================================================================================
# DecolandoState — estado da DroneFSM
# =================================================================================================
# DroneFSMDescription.DECOLANDO
#
# Significado: manobra de decolagem em curso. O drone sobe verticalmente até atingir a
# altitude alvo definida no TAKEOFF. A subida em si é executada pela DeslocamentoFSM
# (estado DESLOCANDO com target apenas em Z); este estado lifecycle apenas observa
# a altitude atual e transiciona para EM_VOO ao atingir o alvo.
#
# Transições:
#     Altitude atual chegou à altitude alvo (tolerância) → EM_VOO.
#     Drone desarmado em meio à decolagem → POUSADO_DESARMADO.
#     PX4 saiu do modo OFFBOARD → OFFBOARD_DESATIVADO.
# =================================================================================================

from px4_msgs.msg import VehicleStatus

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class DecolandoState(BaseState):
    """Drone em decolagem vertical até a altitude alvo."""

    def on_enter(self) -> None:
        self.node.get_logger().info(
            f"DECOLAGEM iniciada. Altitude alvo: {self.context.takeoff_altitude:.2f}m."
        )

    def on_step(self):
        # `self.context` é o DroneFSMContext (lifecycle).
        # Variáveis de manobra (position_tolerance) vivem no DeslocamentoFSMContext —
        # acesso via self.node.deslocamento_fsm_context.
        px4 = self.node.state_px4

        # Saiu do OFFBOARD durante a decolagem: aborta.
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return DS.OFFBOARD_DESATIVADO

        # Drone desarmou em pleno voo: falha de segurança, volta ao chão.
        if not px4.is_armed:
            self.node.get_logger().warn("Drone desarmou durante decolagem.")
            return DS.POUSADO_DESARMADO

        # Sem posição local conhecida ainda — aguarda.
        if px4.local_position is None:
            return None

        if self.node.trajectory_profile.is_done() and not px4.is_landed:
            self.node.deslocamento_fsm_context.last_static_position = list(
                self.node.trajectory_profile.target)
            self.node.get_logger().info('Decolagem concluída com velocidade estabilizada.')
            return DS.EM_VOO

        return None
