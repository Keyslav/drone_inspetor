"""Últimas amostras e saúde de tópicos, sem dependência de ROS ou Qt."""

from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from threading import RLock
import time
from types import MappingProxyType
from typing import Any


@dataclass(frozen=True, slots=True)
class MonitorTopic:
    """Metadados para exibição; os testes conferem os nomes contra as specs ROS."""

    key: str
    label: str
    topic: str
    stale_after: float


COORDINATE_NOTES = {
    'drone': ('Contrato legado: posições locais Norte/Leste/Cima; velocidade e aceleração '
              'Norte/Leste/Abaixo. Yaw: 0° Norte, 90° Leste. Altitude GPS: AMSL.'),
    'mission': 'TAKEOFF: altura acima do HOME; alt dos destinos GOTO: altitude absoluta AMSL.',
    'status': 'Estados PX4; não contém coordenadas de posição.',
    'local': ('PX4 NED: X Norte, Y Leste, Z Abaixo; origem do estimador, não necessariamente '
              'o HOME nem a origem do Gazebo. Velocidade e aceleração também NED.'),
    'setpoint': ('Referência PX4 NED, mesma origem da posição local. Yaw em radianos: '
                 '0 Norte, +π/2 Leste. NaN indica componente não especificada.'),
    'lidar': 'Scan no frame do sensor: ângulos anti-horários, zero à frente; +90° à esquerda.',
    'down': 'Distância ao solo ao longo do feixe; não é altitude AMSL nem Z NED.',
    'depth': 'Scan horizontal FLU: Frente/Esquerda/Cima, após projeção da câmera óptica.',
}


MONITOR_TOPICS = (
    MonitorTopic('drone', 'Drone', '/drone_inspetor/interno/drone_node/drone_state', 2.),
    MonitorTopic('mission', 'Missão', '/drone_inspetor/interno/mission_node/mission_state', 3.),
    MonitorTopic('status', 'Estado PX4', '/fmu/out/vehicle_status_v1', 1.),
    MonitorTopic('local', 'Posição local NED', '/fmu/out/vehicle_local_position', 1.),
    MonitorTopic('battery', 'Bateria', '/fmu/out/battery_status', 3.),
    MonitorTopic('setpoint', 'Referência NED', '/fmu/in/trajectory_setpoint', 0.5),
    MonitorTopic('lidar', 'LiDAR horizontal', '/drone_inspetor/externo/lidar/scan', 1.),
    MonitorTopic('down', 'LiDAR inferior', '/drone_inspetor/externo/lidar_down/scan', 1.),
    MonitorTopic('depth', 'Profundidade', '/drone_inspetor/interno/depth_node/scan', 1.),
)


@dataclass(frozen=True, slots=True)
class TopicSnapshot:
    """Snapshot independente: a GUI não recebe o buffer mutável dos callbacks."""

    values: Mapping[str, Any]
    age: float | None
    hz: float
    health: str
    topic: str
    label: str


def _freeze(value):
    """Normaliza arrays numéricos e containers recursivamente sem importar NumPy."""
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (str, bytes, int, float, bool, type(None))):
        return value
    if hasattr(value, 'tolist'):
        return _freeze(value.tolist())
    if isinstance(value, Sequence):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(_freeze(item) for item in value)
    raise TypeError(f'Tipo de telemetria não suportado: {type(value).__name__}')


class MonitorStore:
    """Armazena um payload por tópico e uma janela de recepção local monotônica.

    Idade/frequência medem chegada ao monitor, não o timestamp de origem da
    mensagem. Republicar um estado PX4 antigo conta como uma nova recepção.
    """

    def __init__(self, *, clock=time.monotonic, frequency_window=2., history_size=256):
        if not math.isfinite(frequency_window) or frequency_window <= 0:
            raise ValueError('frequency_window deve ser positivo e finito')
        if not isinstance(history_size, int) or isinstance(history_size, bool) or history_size < 2:
            raise ValueError('history_size deve ser um inteiro maior ou igual a 2')
        self._clock = clock
        self._frequency_window = frequency_window
        self._lock = RLock()
        self._topics = {topic.key: topic for topic in MONITOR_TOPICS}
        self._topic_names = {topic.key: topic.topic for topic in MONITOR_TOPICS}
        self._values = {key: MappingProxyType({}) for key in self._topics}
        self._arrivals = {key: deque(maxlen=history_size) for key in self._topics}

    def set_topic_name(self, key, resolved_name):
        """Atualiza o nome efetivo retornado pela subscription, incluindo remaps."""
        if not isinstance(resolved_name, str) or not resolved_name:
            raise ValueError('resolved_name deve ser um nome de tópico não vazio')
        with self._lock:
            if key not in self._topics:
                raise KeyError(key)
            self._topic_names[key] = resolved_name

    def _now(self, now):
        instant = self._clock() if now is None else now
        if not math.isfinite(instant):
            raise ValueError('O instante monotônico deve ser finito')
        return float(instant)

    def update(self, key, values, now=None):
        """Copia a amostra e registra recepção; NaN de sensor é preservado como dado."""
        if not isinstance(values, Mapping):
            raise TypeError('values deve ser um mapping')
        frozen = _freeze(values)
        with self._lock:
            instant = self._now(now)
            arrivals = self._arrivals[key]
            if arrivals and instant < arrivals[-1]:
                arrivals.clear()
            self._values[key] = frozen
            arrivals.append(instant)
            while arrivals and instant - arrivals[0] > self._frequency_window:
                arrivals.popleft()

    def snapshot(self, now=None):
        """Calcula frequência recente e expiração sem depender de novas mensagens."""
        snapshots = {}
        with self._lock:
            instant = self._now(now)
            for key, topic in self._topics.items():
                arrivals = self._arrivals[key]
                age = None if not arrivals else max(0., instant - arrivals[-1])
                health = 'waiting' if age is None else (
                    'live' if age <= topic.stale_after else 'stale'
                )
                recent = [stamp for stamp in arrivals
                          if 0 <= instant - stamp <= self._frequency_window]
                span = recent[-1] - recent[0] if len(recent) >= 2 else 0.
                hz = (len(recent) - 1) / span if span > 0 else 0.
                snapshots[key] = TopicSnapshot(
                    self._values[key], age, hz, health, self._topic_names[key], topic.label,
                )
        return snapshots
