"""Solicita detecção do alvo atual e decide usando seu resultado identificado."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class ExecutandoDetectandoState(BaseState):
    """Detecção negativa ou expirada pula o ponto, sem aceitar resposta atrasada."""

    def on_enter(self):
        """Prepara uma detecção pertencente ao ponto atual."""
        self.operation = None

    def on_step(self):
        """Avança para gravação somente com uma detecção válida."""
        point = self.context.get_ponto_atual()
        if point is None or point.inspection is None:
            return MS.EXECUTANDO_INSPECIONANDO
        if self.operation is None:
            self.operation = self.node.cv.request_detection(point.inspection)
        if not self.operation.done:
            return None
        if self.operation.success:
            return MS.EXECUTANDO_INSPECIONANDO_ESCANEANDO
        self.node.get_logger().warning(f'Detecção: {self.operation.message}; avançando ponto')
        self.context.advance_waypoint()
        return MS.EXECUTANDO_INSPECIONANDO

    def on_exit(self):
        """Invalida qualquer resposta que chegar depois desta fase."""
        self.node.cv.cancel_detection()
