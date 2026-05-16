# desativado.py
# =================================================================================================
# ESTADO: DESATIVADO
# =================================================================================================
# Aguarda o drone sair do modo OFFBOARD_DESATIVADO para transicionar para PRONTO.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class DesativadoState(BaseState):
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
