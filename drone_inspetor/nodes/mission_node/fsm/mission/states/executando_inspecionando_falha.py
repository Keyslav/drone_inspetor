"""Falha operacional encerra a inspeção e inicia o retorno."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class ExecutandoFalhaState(BaseState):
    """Evita permanecer indefinidamente no antigo estado placeholder de falha."""

    def on_step(self):
        """Registra o motivo da interrupção e inicia retorno."""
        self.node.get_logger().error(f'Inspeção interrompida: {self.context.failure_reason}')
        return MS.RETORNANDO
