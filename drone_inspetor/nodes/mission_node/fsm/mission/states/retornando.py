"""Solicita RTL com retentativa limitada somente após rejeição confirmada."""

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.action_client import ActionStatus
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class RetornandoState(BaseState):
    """Espera cancelamento anterior e nunca reenvia um RTL de aceitação incerta."""

    def on_enter(self):
        """Prepara o limite monotônico, iniciado na primeira tentativa de RTL."""
        self.operation = None
        self.acceptance_deadline = None
        self.retry_at = None
        self.failure_logged = False

    def on_step(self):
        """Retenta rejeições temporárias; falhas após aceitação exigem supervisão."""
        drone = self.node.drone
        if drone.is_landed and not drone.is_armed:
            self.node.actions.cancel()
            return MS.DESATIVADO
        if self.node.actions.busy:
            return None
        now = self.node.monotonic_time()
        if self.operation is None:
            # O servidor pode ainda estar freando após o prazo de cancelamento local.
            self.acceptance_deadline = now + self.node.config.return_acceptance_timeout
            self._request_return(now)
            return None
        if not self.operation.done or self.operation.result.success:
            return None
        status = self.operation.result.status
        rejected = status is ActionStatus.REJECTED and self.operation.goal_handle is None
        if rejected and now < self.acceptance_deadline:
            if now >= self.retry_at:
                self._request_return(now)
            return None
        if not self.failure_logged:
            self.failure_logged = True
            reason = ('Prazo de aceitação do RTL excedido após rejeições do servidor'
                      if rejected else self.operation.result.message)
            self.context.failure_reason = reason
            self.node.get_logger().error(
                f'Retorno não confirmado: {reason}. '
                'Aguardando pouso ou intervenção do operador.')
        return None

    def _request_return(self, now):
        self.operation = self.node.actions.return_home()
        self.retry_at = now + self.node.config.return_retry_interval
