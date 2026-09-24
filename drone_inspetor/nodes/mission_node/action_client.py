"""Ciclo de vida de comandos; callbacks pertencem a uma operação identificada."""

import math
import time
from dataclasses import dataclass
from enum import Enum
from itertools import count
from threading import RLock


class ActionStatus(Enum):
    """Resultado terminal de uma operação de voo."""

    SUCCEEDED = 'concluída'
    FAILED = 'falhou'
    REJECTED = 'rejeitada antes da aceitação'
    CANCELED = 'cancelada'
    TIMED_OUT = 'prazo excedido'


@dataclass(frozen=True)
class ActionResult:
    """Resultado estável, sem dependência do objeto ROS recebido."""

    status: ActionStatus
    message: str

    @property
    def success(self):
        """Indica conclusão confirmada pelo servidor e sem cancelamento local."""
        return self.status is ActionStatus.SUCCEEDED


@dataclass
class FlightOperation:
    """Identidade e progresso de um comando, nunca reutilizados por outro ponto."""

    operation_id: int
    command: str
    started_at: float
    last_feedback_at: float
    result: ActionResult | None = None
    cancel_requested_at: float | None = None
    cancel_status: ActionStatus = ActionStatus.CANCELED
    goal_handle: object = None

    @property
    def done(self):
        """Indica que a operação já possui resultado terminal."""
        return self.result is not None


def build_goal(command, **parameters):
    """Constrói o contrato ROS somente nesta borda; omissões são NaN explícitos."""
    from drone_inspetor_msgs.action import DroneCommand
    goal = DroneCommand.Goal()
    goal.command = command
    for field in ('lat', 'lon', 'alt', 'yaw', 'focus_lat', 'focus_lon', 'altitude'):
        value = parameters.get(field)
        setattr(goal, field, float(value) if value is not None else math.nan)
    goal.use_focus = bool(parameters.get('use_focus', False))
    return goal


class DroneActionClient:
    """Reserva uma operação até resultado/cancelamento; prazos são monotônicos."""

    def __init__(self, client, logger, *, feedback_timeout=60.0, cancel_timeout=5.0,
                 clock=time.monotonic, goal_factory=build_goal):
        """Recebe transporte e relógio explícitos para supervisionar cada operação."""
        self._client = client
        self._logger = logger
        self._clock = clock
        self._goal_factory = goal_factory
        self._feedback_timeout = feedback_timeout
        self._cancel_timeout = cancel_timeout
        self._ids = count(1)
        self._active = None
        self._lock = RLock()

    @property
    def active(self):
        """Retorna a operação reservada, inclusive enquanto aguarda cancelamento."""
        with self._lock:
            return self._active

    @property
    def busy(self):
        """Impede envio concorrente enquanto o servidor ainda possui o comando."""
        return self.active is not None

    def start(self, command, **parameters):
        """Envia sem bloquear o executor; indisponibilidade produz falha explícita."""
        with self._lock:
            if self._active is not None:
                return None
            now = self._clock()
            operation = FlightOperation(next(self._ids), command, now, now)
            self._active = operation
            self._logger.info(f'Comando {operation.operation_id} {command}: enviado: {parameters}')
            try:
                if not self._client.server_is_ready():
                    self._finish(operation, ActionStatus.FAILED, 'Servidor de voo indisponível')
                    return operation
                future = self._client.send_goal_async(
                    self._goal_factory(command, **parameters),
                    feedback_callback=lambda message: self._feedback(operation, message))
                future.add_done_callback(lambda response: self._accepted(operation, response))
            except Exception as error:
                self._finish(operation, ActionStatus.FAILED, f'Falha enviando {command}: {error}')
            return operation

    def arm(self):
        """Solicita armamento."""
        return self.start('ARM')

    def takeoff(self, altitude_m):
        """Solicita decolagem até altitude relativa ao home, em metros."""
        return self.start('TAKEOFF', altitude=altitude_m)

    def navigate_to(self, waypoint):
        """Navega para um waypoint validado preservando foco e unidades públicas."""
        return self.start(
            'GOTO', lat=waypoint.latitude_deg, lon=waypoint.longitude_deg,
            alt=waypoint.altitude_m, yaw=waypoint.yaw_deg,
            use_focus=waypoint.focus_latitude_deg is not None,
            focus_lat=waypoint.focus_latitude_deg, focus_lon=waypoint.focus_longitude_deg)

    def return_home(self):
        """Delega o retorno completo ao controlador de voo."""
        return self.start('RTL')

    def _accepted(self, operation, future):
        with self._lock:
            try:
                handle = future.result()
                if not handle.accepted:
                    status = (operation.cancel_status
                              if operation.cancel_requested_at is not None
                              else ActionStatus.REJECTED)
                    self._finish(operation, status, 'Comando rejeitado pelo servidor')
                    return
                operation.goal_handle = handle
                if self._active is not operation or operation.done:
                    # O prazo local pode terminar antes de o servidor aceitar o goal.
                    handle.cancel_goal_async()
                    return
                result_future = handle.get_result_async()
                result_future.add_done_callback(lambda result: self._result(operation, result))
                if operation.cancel_requested_at is not None:
                    self._send_cancel(operation)
            except Exception as error:
                self._finish(operation, ActionStatus.FAILED, f'Falha aceitando comando: {error}')

    def _feedback(self, operation, message):
        with self._lock:
            if self._active is operation and not operation.done:
                operation.last_feedback_at = self._clock()

    def _result(self, operation, future):
        with self._lock:
            if self._active is not operation or operation.done:
                return
            try:
                response = future.result()
                result = response.result
                # GoalStatus.STATUS_SUCCEEDED == 4; não aceitar sucesso de um goal abortado.
                succeeded = result.success and getattr(response, 'status', 4) == 4
                status = ActionStatus.SUCCEEDED if succeeded else ActionStatus.FAILED
                if operation.cancel_requested_at is not None:
                    status = operation.cancel_status
                self._finish(operation, status, result.message)
            except Exception as error:
                self._finish(operation, ActionStatus.FAILED, f'Falha recebendo resultado: {error}')

    def _finish(self, operation, status, message):
        if self._active is not operation or operation.done:
            return
        operation.result = ActionResult(status, message)
        self._active = None
        self._logger.info(
            f'Comando {operation.operation_id} {operation.command}: {status.value}: {message}')

    def cancel(self, *, timed_out=False):
        """Inclui goals ainda não aceitos e mantém a reserva até resultado ou prazo."""
        with self._lock:
            operation = self._active
            if operation is None:
                return False
            if operation.cancel_requested_at is None:
                operation.cancel_requested_at = self._clock()
                operation.cancel_status = (
                    ActionStatus.TIMED_OUT if timed_out else ActionStatus.CANCELED)
                if operation.goal_handle is not None:
                    self._send_cancel(operation)
            return True

    def _send_cancel(self, operation):
        try:
            future = operation.goal_handle.cancel_goal_async()
            future.add_done_callback(lambda response: self._cancel_response(operation, response))
        except Exception as error:
            self._logger.error(f'Falha solicitando cancelamento: {error}')

    def _cancel_response(self, operation, future):
        with self._lock:
            if self._active is not operation:
                return
            try:
                if not future.result().goals_canceling:
                    self._logger.warning(
                        f'Cancelamento do comando {operation.operation_id} não aceito')
            except Exception as error:
                self._logger.error(f'Falha recebendo cancelamento: {error}')

    def poll(self):
        """Aplica prazos mesmo com /clock pausado ou antes da primeira resposta."""
        with self._lock:
            operation = self._active
            if operation is None:
                return
            now = self._clock()
            if operation.cancel_requested_at is not None:
                if now - operation.cancel_requested_at >= self._cancel_timeout:
                    self._finish(operation, operation.cancel_status,
                                 'Sem confirmação terminal de cancelamento; '
                                 'operação invalidada localmente')
            elif now - operation.last_feedback_at >= self._feedback_timeout:
                self.cancel(timed_out=True)
