"""Confirma encerramento da inspeção antes de avançar para outro waypoint."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class ExecutandoEscaneamentoFinalizadoState(BaseState):
    """Falha de stop é explícita e interrompe inspeção em vez de misturar sessões."""

    def on_enter(self):
        """Solicita parada de gravação e de anomalias."""
        self.operations = self.node.cv.stop_inspection()

    def on_step(self):
        """Avança o ponto somente após confirmar o encerramento dos recursos."""
        if not all(operation.done for operation in self.operations):
            return None
        failed = next((operation for operation in self.operations if not operation.success), None)
        if failed is not None:
            self.context.failure_reason = failed.message
            return MS.EXECUTANDO_INSPECIONANDO_FALHA
        self.context.advance_waypoint()
        return MS.EXECUTANDO_INSPECIONANDO
