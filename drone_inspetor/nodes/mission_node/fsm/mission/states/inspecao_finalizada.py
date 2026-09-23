"""Todos os pontos concluídos: iniciar retorno."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class InspecaoFinalizadaState(BaseState):
    """Transição explícita de fim de inspeção para retorno."""

    def on_step(self):
        """Encaminha fim de inspeção para o ciclo de retorno."""
        return MS.RETORNANDO
