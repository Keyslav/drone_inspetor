# desativado.py
# =================================================================================================
# ESTADO: DESATIVADO
# =================================================================================================
# Aguarda o drone sair do modo OFFBOARD_DESATIVADO para transicionar para PRONTO.
# =================================================================================================

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import MissionStateDescription as MS, DroneStateDescription as DS


class DesativadoState(State):
    """
    Estado inicial. Sistema desativado, aguardando drone estar pousado e desarmado.
    """

    def on_step(self):
        drone_state = self.node.drone.state

        if drone_state == DS.OFFBOARD_DESATIVADO:
            self.node.get_logger().info(
                "Drone fora do modo OFFBOARD. Aguardando ativação...",
                throttle_duration_sec=5.0,
            )
            return None

        if drone_state == DS.POUSADO_DESARMADO:
            return MS.PRONTO

        self.node.get_logger().info(
            f"Aguardando drone estar pousado e desarmado. Estado atual: {drone_state.name}",
            throttle_duration_sec=5.0,
        )
        return None
