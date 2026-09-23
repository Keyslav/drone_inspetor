"""Resultados de inferência com idade limitada e espera operacional interrompível."""

from copy import deepcopy
import math
from threading import Condition
import time


class DetectionBuffer:
    """Entrega somente resultados de frames recebidos após iniciar a consulta.

    Recepção, idade e timeout usam tempo monotônico, independente de /clock.
    Um frame cuja inferência demorou demais não conta como uma observação atual.
    """

    def __init__(self, max_frame_age=1.0):
        if not math.isfinite(max_frame_age) or max_frame_age <= 0:
            raise ValueError('Idade máxima do frame deve ser positiva')
        self.max_frame_age = max_frame_age
        self._condition = Condition()
        self._detections = []
        self._received_at = float('-inf')
        self._closed = False
        self._generation = 0

    def publish(self, detections, received_at):
        """Publica um snapshot e acorda consultas sem compartilhar listas mutáveis."""
        with self._condition:
            if not self._closed:
                self._detections = deepcopy(detections)
                self._received_at = received_at
                self._condition.notify_all()

    def wait_for(self, object_name, timeout):
        """Retorna a melhor observação atual ou None em timeout/shutdown."""
        if not math.isfinite(timeout) or timeout <= 0 or not object_name.strip():
            raise ValueError('Objeto e timeout positivo são obrigatórios')
        requested_at = time.monotonic()
        deadline = requested_at + timeout
        target = object_name.casefold().strip()
        with self._condition:
            generation = self._generation
            while not self._closed and generation == self._generation:
                now = time.monotonic()
                if now >= deadline:
                    break
                age = now - self._received_at
                if self._received_at >= requested_at and 0 <= age <= self.max_frame_age:
                    candidates = [
                        detection for detection in self._detections
                        if any(target in detection.get(key, '').casefold()
                               for key in ('class', 'object_type'))
                    ]
                    if candidates:
                        return deepcopy(max(candidates, key=lambda item: item['confidence']))
                self._condition.wait(deadline - now)
        return None

    def invalidate(self):
        """Descarta resultados ao trocar missão ou modelos."""
        with self._condition:
            self._detections = []
            self._received_at = float('-inf')
            self._generation += 1
            self._condition.notify_all()

    def close(self):
        """Interrompe imediatamente todos os serviços que aguardam resultados."""
        with self._condition:
            self._closed = True
            self._condition.notify_all()
