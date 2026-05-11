# offboard_desativado.py
# Estado: OFFBOARD_DESATIVADO
# Drone fora do modo offboard (manual, POSCTL, etc.).
# Aguarda retorno ao modo OFFBOARD para decidir o próximo estado.

from px4_msgs.msg import VehicleStatus

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class OffboardDesativadoState(State):
    """
    Drone fora do modo offboard.

    Monitora nav_state do PX4. Quando retorna ao modo OFFBOARD,
    decide o próximo estado com base em is_armed e is_landed.
    """

    def on_step(self):
        if self.context.state_px4.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.context.state_px4.is_armed:
                if self.context.state_px4.is_landed:
                    return DS.POUSADO_ARMADO
                else:
                    return DS.VOANDO_PRONTO
            else:
                return DS.POUSADO_DESARMADO
        return None
