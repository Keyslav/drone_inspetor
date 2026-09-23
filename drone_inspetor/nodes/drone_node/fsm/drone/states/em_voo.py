"""Voo ativo, incluindo a transferência intencional para LAND/RTL do PX4."""

from px4_msgs.msg import VehicleStatus

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class EmVooState(BaseState):
    """Acompanha o estado físico sem confundir AUTO comandado com perda de controle."""

    def on_enter(self):
        self.node.deslocamento_fsm_context.store_static_position()
        self.node.get_logger().info('Drone EM_VOO')

    def on_step(self):
        px4 = self.node.state_px4
        if px4.is_landed:
            return DS.POUSADO_ARMADO if px4.is_armed else DS.POUSADO_DESARMADO
        if not px4.is_armed:
            self.node.get_logger().error('Drone desarmou em pleno voo')
            return DS.EMERGENCIA
        if self.context.accepts_native_mode():
            return None
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return DS.OFFBOARD_DESATIVADO
        if self.context.native_mode_observed:
            # Depois de AUTO confirmado, voltar ao OFFBOARD exige uma nova operação.
            return DS.OFFBOARD_DESATIVADO
        return None
