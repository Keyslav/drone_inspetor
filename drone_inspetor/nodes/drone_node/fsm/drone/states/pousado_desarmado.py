# =================================================================================================
# PousadoDesarmadoState — estado da DroneFSM
# =================================================================================================
# DroneFSMDescription.POUSADO_DESARMADO
#
# Significado: drone no solo, motores desligados. Aguarda comando ARM.
#
# Transições:
#     pending_command == "ARM"  → executa arm e vai para POUSADO_ARMADO (após PX4 confirmar).
#     PX4 saiu do modo OFFBOARD → OFFBOARD_DESATIVADO.
#     Drone está armado (mudou externamente) → POUSADO_ARMADO.
# =================================================================================================

from px4_msgs.msg import VehicleStatus

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class PousadoDesarmadoState(BaseState):
    """Drone no solo, desarmado. Aguarda comando ARM."""

    def on_enter(self) -> None:
        # Limpa qualquer resíduo de trajetória anterior (caso entremos aqui após pouso).
        self.node.deslocamento_fsm_context.reset()
        # Reseta a flag de emergência: novo ciclo de operação pode começar.
        self.context.emergency_active = False
        self.node.get_logger().info("Drone POUSADO_DESARMADO. Aguardando comando ARM.")

    def on_step(self):
        ctx = self.context
        px4 = self.node.state_px4

        # Saiu do OFFBOARD: volta ao estado raiz.
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return DS.OFFBOARD_DESATIVADO

        # Comando ARM pendente: chama o helper PX4 do node e consome o comando.
        # A confirmação física vem via px4.is_armed; a transição acontece nesse mesmo ciclo
        # logo abaixo (ou no próximo, se o PX4 atrasar o ACK).
        if ctx.pending_command == "ARM":
            self.node.arm_drone()
            ctx.pending_command = None
            return None

        # Detectou armamento (físico) — pode ter sido por comando nosso ou externo.
        if px4.is_armed:
            return DS.POUSADO_ARMADO

        return None
