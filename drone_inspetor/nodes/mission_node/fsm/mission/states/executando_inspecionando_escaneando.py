"""Só conta permanência depois de confirmar gravação e análise de anomalias."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class ExecutandoEscaneandoState(BaseState):
    """Permanência acompanha tempo ROS; espera dos serviços usa prazo monotônico."""

    def on_enter(self):
        """Solicita controles e aguarda confirmação antes de contar permanência."""
        self.operations = (self.node.cv.start_recording(), self.node.cv.enable_anomalies(True))
        self.started_at = None

    def on_step(self):
        """Aplica duração ROS e encaminha falhas dos serviços para retorno."""
        failed = next((operation for operation in self.operations
                       if operation.done and not operation.success), None)
        if failed is not None:
            self.context.failure_reason = failed.message
            return MS.EXECUTANDO_INSPECIONANDO_FALHA
        if not all(operation.done for operation in self.operations):
            return None
        now = self.node.now()
        if self.started_at is None or now < self.started_at:
            # Um reset do /clock reinicia a permanência, nunca a conclui artificialmente.
            self.started_at = now
        if now - self.started_at >= self.context.tempo_de_permanencia:
            return MS.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO
        return None
