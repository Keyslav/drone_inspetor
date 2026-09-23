"""Navegação sequencial com resultado associado ao waypoint que enviou o goal."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class ExecutandoInspecionandoState(BaseState):
    """Um resultado antigo nunca marca outro waypoint como alcançado."""

    def on_enter(self):
        """Separa a operação de navegação de resultados anteriores."""
        self.operation = None
        self.waypoint_index = None

    def on_step(self):
        """Só marca chegada quando operação e índice do waypoint coincidem."""
        if self.context.mission is None:
            return MS.RETORNANDO
        point = self.context.get_ponto_atual()
        if point is None:
            return MS.INSPECAO_FINALIZADA
        if self.operation is None:
            self.waypoint_index = self.context.ponto_de_inspecao_indice_atual
            self.operation = self.node.actions.navigate_to(point)
        if self.operation is None or not self.operation.done:
            return None
        if self.waypoint_index != self.context.ponto_de_inspecao_indice_atual:
            self.operation = None
            return None
        if not self.operation.result.success:
            self.context.failure_reason = self.operation.result.message
            return MS.RETORNANDO
        if not self.context.waypoint_reached:
            self.context.waypoint_reached = True
            self.context.ponto_de_inspecao_tempo_de_chegada = self.node.now()
        if point.inspection is not None:
            return MS.EXECUTANDO_INSPECIONANDO_DETECTANDO
        self.context.advance_waypoint()
        self.operation = None
        return None
