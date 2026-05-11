# pousado_desarmado.py
# Estado: POUSADO_DESARMADO
# Drone no chão e desarmado.
# Aguarda armamento para transicionar para POUSADO_ARMADO.

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class PousadoDesarmadoState(State):
    """
    Drone pousado e desarmado.

    Monitora is_armed do PX4. Quando armado, transiciona para POUSADO_ARMADO.
    """

    def on_step(self):
        if self.context.state_px4.is_armed:
            return DS.POUSADO_ARMADO
        return None
