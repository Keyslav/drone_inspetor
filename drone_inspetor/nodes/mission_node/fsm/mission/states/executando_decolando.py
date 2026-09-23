"""Decola até a altitude configurada e aguarda a conclusão confirmada."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class ExecutandoDecolandoState(BaseState):
    """Usa resultado do TAKEOFF, sem inferir conclusão de telemetria intermediária."""

    def on_enter(self):
        """Prepara uma operação de decolagem para esta sessão."""
        self.operation = None

    def on_step(self):
        """Aguarda TAKEOFF confirmado antes de iniciar navegação."""
        if self.operation is None:
            self.operation = self.node.actions.takeoff(self.context.takeoff_altitude)
        if self.operation is None or not self.operation.done:
            return None
        if self.operation.result.success:
            return MS.EXECUTANDO_INSPECIONANDO
        self.context.failure_reason = self.operation.result.message
        return MS.RETORNANDO
