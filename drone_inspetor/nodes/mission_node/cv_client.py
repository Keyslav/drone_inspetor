"""Contratos CV e correlação de respostas; estados não constroem requests ROS."""

import math
import time
from dataclasses import dataclass
from itertools import count
from threading import RLock


def detection_request(target, timeout_s):
    """Usa os nomes públicos de CVDetectionSRV, preservados na versão 2."""
    from drone_inspetor_msgs.srv import CVDetectionSRV
    request = CVDetectionSRV.Request()
    request.object_name = target.object_name
    request.anomaly_types = list(target.anomaly_types)
    request.timeout_seconds = float(timeout_s)
    return request


def recording_request(enabled):
    """Solicita gravação; a pasta pertence ao contrato MissionStateMSG."""
    from drone_inspetor_msgs.srv import RecordDetectionsSRV
    request = RecordDetectionsSRV.Request()
    request.start_recording = bool(enabled)
    return request


def anomaly_request(enabled):
    """Constrói o controle de anomalias no contrato existente."""
    from drone_inspetor_msgs.srv import EnableAnomalyDetectionSRV
    request = EnableAnomalyDetectionSRV.Request()
    request.enable = bool(enabled)
    return request


@dataclass
class CVOperation:
    """Resultado pertencente a uma requisição identificada, sem efeitos na missão."""

    operation_id: int
    started_at: float
    timeout_s: float
    done: bool = False
    success: bool = False
    message: str = ''
    bbox_center: tuple[float, float] | None = None
    dispatched: bool = False

    def finish(self, success, message, bbox_center=None):
        """Aceita um único resultado; respostas após prazo/reset não o reescrevem."""
        if not self.done:
            self.done = True
            self.success = bool(success)
            self.message = str(message)
            self.bbox_center = bbox_center


class _ControlChannel:
    """Serializa start/stop para um serviço: stop sempre sucede start pendente."""

    def __init__(self, client, request_factory):
        self.client = client
        self.request_factory = request_factory
        self.desired = None
        self.pending = None
        self.waiting = []


class CVClient:
    """Coordena detecção e controles assíncronos com prazos de tempo real."""

    def __init__(self, detection_client, record_client, anomaly_client, logger, *,
                 detection_timeout=10.0, service_timeout=2.0, control_timeout=5.0,
                 clock=time.monotonic):
        """Recebe clientes existentes, sem criar canais ou nós adicionais."""
        self._detection_client = detection_client
        self._record = _ControlChannel(record_client, recording_request)
        self._anomaly = _ControlChannel(anomaly_client, anomaly_request)
        self._logger = logger
        self._clock = clock
        self._detection_timeout = detection_timeout
        self._service_timeout = service_timeout
        self._control_timeout = control_timeout
        self._ids = count(1)
        self._detection = None
        self._lock = RLock()

    def _operation(self, timeout):
        return CVOperation(next(self._ids), self._clock(), timeout)

    def request_detection(self, target):
        """Invalida a resposta anterior antes de iniciar a inspeção do novo ponto."""
        with self._lock:
            self.cancel_detection()
            operation = self._operation(self._detection_timeout)
            self._detection = operation
            try:
                if not self._detection_client.service_is_ready():
                    operation.finish(False, 'Serviço de detecção indisponível')
                    return operation
                future = self._detection_client.call_async(
                    detection_request(target, self._service_timeout))
                future.add_done_callback(
                    lambda response: self._detection_done(operation, response))
            except Exception as error:
                operation.finish(False, f'Falha solicitando detecção: {error}')
            return operation

    def _detection_done(self, operation, future):
        with self._lock:
            if self._detection is not operation or operation.done:
                return
            try:
                response = future.result()
                center = tuple(response.bbox_center[:2])
                valid = response.success and len(center) == 2 and all(map(math.isfinite, center))
                operation.finish(valid, response.message, center if valid else None)
            except Exception as error:
                operation.finish(False, f'Falha recebendo detecção: {error}')

    def cancel_detection(self):
        """O serviço não oferece cancelamento remoto; invalida o efeito da resposta."""
        with self._lock:
            if self._detection is not None:
                self._detection.finish(False, 'Detecção invalidada')
                self._detection = None

    def start_recording(self):
        """Solicita início de gravação e permite verificar confirmação."""
        return self._control(self._record, True)

    def stop_recording(self):
        """Enfileira parada após eventual start ainda em andamento."""
        return self._control(self._record, False)

    def enable_anomalies(self, enabled):
        """Solicita ativação/desativação sem acessar o cliente ROS nos estados."""
        return self._control(self._anomaly, bool(enabled))

    def _control(self, channel, enabled):
        with self._lock:
            if channel.desired is not None and channel.desired[0] == enabled:
                current = channel.desired[1]
                if not current.done or current.success:
                    return current
            if channel.desired is not None:
                channel.desired[1].finish(False, 'Controle substituído por nova solicitação')
            operation = self._operation(self._control_timeout)
            channel.desired = (enabled, operation)
            channel.waiting.append((enabled, operation))
            self._dispatch_control(channel)
            return operation

    def _dispatch_control(self, channel):
        if channel.pending is not None:
            return
        while channel.waiting:
            enabled, operation = channel.waiting.pop(0)
            # Starts expirados/substituídos não devem produzir efeitos tardios.
            # Stops são barreiras: precisam executar mesmo após o prazo local.
            if operation.done and enabled:
                continue
            try:
                if not channel.client.service_is_ready():
                    operation.finish(False, 'Serviço de controle CV indisponível')
                    continue
                operation.dispatched = True
                channel.pending = (enabled, operation)
                future = channel.client.call_async(channel.request_factory(enabled))
                future.add_done_callback(
                    lambda response: self._control_done(channel, operation, response))
                return
            except Exception as error:
                channel.pending = None
                operation.finish(False, f'Falha solicitando controle CV: {error}')

    def _control_done(self, channel, operation, future):
        with self._lock:
            try:
                response = future.result()
                operation.finish(response.success, response.message)
            except Exception as error:
                operation.finish(False, f'Falha recebendo controle CV: {error}')
            channel.pending = None
            # Mesmo uma resposta de start invalidada exige enviar o stop mais recente.
            self._dispatch_control(channel)

    def stop_inspection(self):
        """Encerra os três fluxos; pode ser chamado em reset, cancelamento e shutdown."""
        self.cancel_detection()
        return self.stop_recording(), self.enable_anomalies(False)

    def poll(self):
        """Aplica prazo operacional monotônico independentemente do relógio ROS."""
        with self._lock:
            now = self._clock()
            operations = [self._detection]
            for channel in (self._record, self._anomaly):
                if channel.desired is not None:
                    operations.append(channel.desired[1])
                if channel.pending is not None:
                    operations.append(channel.pending[1])
                operations.extend(operation for _, operation in channel.waiting)
            for operation in operations:
                if operation is not None and not operation.done:
                    if now - operation.started_at >= operation.timeout_s:
                        operation.finish(False, 'Prazo da operação CV excedido')
                        self._logger.warning(operation.message)
