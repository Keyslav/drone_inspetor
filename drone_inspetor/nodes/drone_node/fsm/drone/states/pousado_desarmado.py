"""Estado final de solo; uma nova ativação OFFBOARD inicia outro ciclo."""

from px4_msgs.msg import VehicleStatus

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class PousadoDesarmadoState(BaseState):
    """Mantém o resultado de RTL/pouso observável até a próxima ativação."""

    def on_enter(self):
        self.node.deslocamento_fsm_context.reset()
        if self.node.state_px4.is_landed and not self.node.state_px4.is_armed:
            self.node.trajectory.reset()
        self.context.emergency_active = False

    def on_step(self):
        context = self.context
        px4 = self.node.state_px4
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if context.native_command is not None and px4.is_landed and not px4.is_armed:
                return None
            return DS.OFFBOARD_DESATIVADO
        context.native_command = None
        context.native_mode_observed = False
        if context.pending_command == 'ARM':
            self.node.arm_drone()
            context.pending_command = None
        if px4.is_armed:
            return DS.POUSADO_ARMADO
        return None
