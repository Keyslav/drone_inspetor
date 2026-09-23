"""Arma uma única vez e verifica o resultado da operação correspondente."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class ExecutandoArmandoState(BaseState):
    """Rejeição/falha encerra a sessão em vez de repetir ARM indefinidamente."""

    def on_enter(self):
        """Descarta referências da sessão anterior."""
        self.operation = None

    def on_step(self):
        """Envia ARM uma vez e decide pelo resultado da mesma operação."""
        if self.operation is None:
            self.operation = self.node.actions.arm()
        if self.operation is None or not self.operation.done:
            return None
        if self.operation.result.success:
            return MS.EXECUTANDO_DECOLANDO
        self.node.get_logger().error(self.operation.result.message)
        return MS.DESATIVADO
