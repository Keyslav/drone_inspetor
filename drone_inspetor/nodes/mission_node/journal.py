"""Registro JSONL de uma sessão; não depende de ROS nem de tempo simulado."""

from datetime import datetime, timezone
import json
from pathlib import Path
import time


class MissionJournal:
    """Eventos pequenos, gravados e descarregados por linha para análise posterior."""

    def __init__(self, on_error, clock=time.monotonic):
        self._on_error = on_error
        self._clock = clock
        self._stream = None
        self._last_snapshot = None
        self._last_state = None
        self._started = 0.0

    def start(self, directory, definition, config, ros_time):
        self.close()
        self._last_snapshot = self._last_state = None
        self._started = self._clock()
        try:
            self._stream = (Path(directory) / 'events.jsonl').open(
                'a', encoding='utf-8', buffering=1)
        except OSError as error:
            self._on_error(f'Não foi possível abrir diário da missão: {error}')
            return
        self.record('session_start', ros_time, definition=definition, config=config)

    def record(self, event, ros_time, **data):
        if self._stream is None:
            return
        row = dict(event=event, utc=datetime.now(timezone.utc).isoformat(),
                   elapsed_s=self._clock() - self._started, ros_time_s=ros_time, **data)
        try:
            self._stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        except (OSError, ValueError, TypeError) as error:
            self.close()
            self._on_error(f'Diário da missão interrompido: {error}')

    def observe(self, ros_time, mission, drone, failure_reason, telemetry_age_s):
        now = self._clock()
        state = (mission['state_name'], drone['state_name'], failure_reason,
                 mission['ponto_de_inspecao_indice_atual'])
        changed = state != self._last_state
        if changed or self._last_snapshot is None or now - self._last_snapshot >= 1.0:
            self.record('state_change' if changed else 'snapshot', ros_time,
                        mission=mission, drone=drone, failure_reason=failure_reason,
                        telemetry_age_s=telemetry_age_s)
            self._last_snapshot, self._last_state = now, state

    def close(self):
        if self._stream is not None:
            stream, self._stream = self._stream, None
            try:
                stream.close()
            except OSError:
                pass
