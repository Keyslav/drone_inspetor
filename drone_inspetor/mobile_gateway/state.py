"""Snapshots limitados e JSON estrito compartilhados entre ROS e HTTP."""

import math
import time
from collections import deque
from collections.abc import Mapping
from datetime import datetime, timezone
from threading import RLock

from drone_inspetor.gui.presentation.telemetry import MonitorStore


def json_value(value):
    """Normaliza NaN/Inf como null, preservando a ausência de leitura do sensor."""
    if isinstance(value, Mapping):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if hasattr(value, 'tolist'):
        return json_value(value.tolist())
    return value


class MobileState:
    """Retém somente a última imagem e amostra; não acumula vídeo na rede."""

    def __init__(self, *, demo=False, clock=time.monotonic):
        """Prepara buffers limitados usando idade de recepção monotônica."""
        self.demo = demo
        self.clock = clock
        self.monitor = MonitorStore(clock=clock)
        self.lock = RLock()
        self.frames = {}
        self.extra = {}
        self.radar = ([], None)
        self.events = deque(maxlen=150)

    def event(self, message, level='info'):
        """Adiciona uma mensagem curta ao histórico recente."""
        with self.lock:
            self.events.append({
                'time': datetime.now(timezone.utc).isoformat(),
                'level': level, 'message': str(message)[:2000],
            })

    def update(self, key, values):
        """Atualiza uma amostra do monitor compartilhado com o dashboard."""
        self.monitor.update(key, values)

    def update_extra(self, key, values, *, label=None, topic=''):
        """Registra telemetria adicional com sua hora local de recepção."""
        with self.lock:
            self.extra[key] = (dict(values), self.clock(), label or key, topic)

    def update_radar(self, points):
        """Retém até 720 retornos válidos para o radar móvel."""
        finite = []
        for distance, angle in points:
            if math.isfinite(distance) and distance > 0 and math.isfinite(angle):
                finite.append([float(distance), float(angle)])
        with self.lock:
            self.radar = (finite[:720], self.clock())

    def put_frame(self, key, data):
        """Substitui a última imagem de um dos três canais conhecidos."""
        if key not in ('camera', 'cv', 'depth') or len(data) > 4 * 1024 * 1024:
            return
        with self.lock:
            self.frames[key] = (bytes(data), self.clock())

    def frame(self, key):
        """Retorna apenas imagens recebidas nos últimos três segundos."""
        item = self.frame_sample(key)
        return item[0] if item is not None else None

    def frame_sample(self, key):
        """Imagem e instante permitem ao vídeo reutilizar a decodificação, sem fila."""
        with self.lock:
            item = self.frames.get(key)
            if item is None or self.clock() - item[1] > 3.0:
                return None
            return item

    def snapshot(self, *, commands_enabled=False, capabilities=()):
        """Monta um estado JSON com idade e saúde dos produtores."""
        if self.demo:
            self._demo_sample()
        topics = {
            key: {'values': json_value(sample.values), 'age_s': sample.age,
                  'hz': sample.hz, 'health': sample.health,
                  'label': sample.label, 'topic': sample.topic}
            for key, sample in self.monitor.snapshot().items()
        }
        with self.lock:
            now = self.clock()
            for key, (values, at, label, topic) in self.extra.items():
                age = max(0.0, now - at)
                topics[key] = {'values': json_value(values), 'age_s': age,
                               'health': 'live' if age <= 3 else 'stale',
                               'hz': None, 'label': label, 'topic': topic}
            points, at = self.radar
            radar = {'points': list(points),
                     'age_s': None if at is None else max(0.0, now - at)}
            frames = {}
            for key in ('camera', 'cv', 'depth'):
                sample = self.frames.get(key)
                age = None if sample is None else max(0.0, now - sample[1])
                frames[key] = {'age_s': age, 'available': age is not None and age <= 3.0}
            events = list(self.events)
        return {'version': 1, 'mode': 'demo' if self.demo else 'ros',
                'commands_enabled': commands_enabled and not self.demo,
                'capabilities': list(capabilities), 'topics': topics,
                'radar': radar, 'frames': frames, 'events': events}

    def _demo_sample(self):
        """Dados identificados como demonstração, sem importar ou publicar ROS."""
        t = self.clock()
        self.update('drone', {
            'state_name': 'DEMONSTRAÇÃO', 'is_armed': False, 'is_landed': True,
            'current_latitude': -22.633890, 'current_longitude': -40.093330,
            'current_altitude': 57.0, 'current_yaw_deg': 20.0,
            'current_local_z': 0.0, 'current_velocity_x': 0.0,
            'current_velocity_y': 0.0, 'current_velocity_z': 0.0,
        })
        self.update('mission', {
            'state_name': 'PRONTO', 'on_mission': False,
            'mission_name': 'Flare', 'total_pontos_de_inspecao': 3,
            'ponto_de_inspecao_indice_atual': 0})
        self.update('status', {
            'nav_state_name': 'DEMO', 'arming_state_name': 'DISARMED', 'failsafe': False})
        self.update('local', {
            'x': 0.0, 'y': 0.0, 'z': 0.0, 'vx': 0.0, 'vy': 0.0,
            'vz': 0.0, 'xy_valid': True, 'z_valid': True})
        self.update('battery', {
            'connected': True, 'remaining': 0.82, 'voltage_v': 16.2, 'current_a': 0.4, 'warning': 0})
        self.update('lidar', {'minimum_distance': 4.3, 'valid_count': 30})
        self.update('down', {'minimum_distance': 0.35, 'valid_count': 1})
        self.update('depth', {'minimum_distance': 5.0, 'valid_count': 120})
        self.update_extra('global', {'lat': -22.633890, 'lon': -40.093330, 'alt': 57.0},
                          label='GPS de demonstração')
        self.update_radar([(4.8 + 0.4 * math.sin(t + i), i * math.pi / 30)
                           for i in range(-12, 13)])
