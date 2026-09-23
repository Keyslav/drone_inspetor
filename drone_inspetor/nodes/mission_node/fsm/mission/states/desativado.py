"""Aguarda telemetria recente e drone disponível para iniciar uma missão."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class DesativadoState(BaseState):
    """Somente o estado pousado/desarmado habilita o início pelo dashboard."""

    def on_step(self):
        """Habilita prontidão somente com pouso, desarme e canal de comandos livre."""
        if self.node.drone.state == DS.POUSADO_DESARMADO and not self.node.actions.busy:
            return MS.PRONTO
        return None
