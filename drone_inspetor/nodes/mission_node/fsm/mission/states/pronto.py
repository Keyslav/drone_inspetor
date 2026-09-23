"""Espera início de uma sessão previamente validada."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class ProntoState(BaseState):
    """Só inicia a missão enquanto o drone continua pousado e desarmado."""

    def on_step(self):
        """Inicia a sessão validada ou desativa se o drone deixou de estar pronto."""
        if self.node.drone.state != DS.POUSADO_DESARMADO:
            return MS.DESATIVADO
        if self.context.on_mission and self.context.mission is not None:
            return MS.EXECUTANDO_ARMANDO
        return None
