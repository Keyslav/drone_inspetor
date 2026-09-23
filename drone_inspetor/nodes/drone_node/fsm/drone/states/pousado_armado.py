"""Estado de solo armado e confirmação do pouso nativo."""

from px4_msgs.msg import VehicleStatus

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class PousadoArmadoState(BaseState):
    """Aguarda decolagem ou auto-desarme após LAND/RTL."""

    def on_enter(self):
        self.node.deslocamento_fsm_context.store_static_position()

    def on_step(self):
        px4 = self.node.state_px4
        context = self.context
        if not px4.is_armed:
            return DS.POUSADO_DESARMADO
        if context.native_command is not None:
            if px4.is_landed:
                return None
            if context.accepts_native_mode():
                return DS.EM_VOO
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return DS.OFFBOARD_DESATIVADO
        if context.pending_command == 'TAKEOFF':
            if not self.node.trajectory.stopped:
                return None
            context.pending_command = None
            return DS.DECOLANDO
        if not px4.is_landed:
            return DS.EM_VOO
        return None
