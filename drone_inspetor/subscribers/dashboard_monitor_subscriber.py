"""Adaptador ROS para o buffer de telemetria consultado periodicamente pela GUI."""

from functools import partial
import math

from px4_msgs.msg import VehicleStatus

from drone_inspetor.common.msg_utils import msg_to_dict
from drone_inspetor.ros_interfaces import Topics, create_subscription_from


MONITOR_SPECS = {
    'drone': Topics.Interno.DRONE_STATE,
    'mission': Topics.Interno.MISSION_STATE,
    'status': Topics.PX4.VEHICLE_STATUS,
    'local': Topics.PX4.VEHICLE_LOCAL_POSITION,
    'battery': Topics.PX4.BATTERY_STATUS,
    'setpoint': Topics.PX4.TRAJECTORY_SETPOINT,
    'lidar': Topics.Externo.LIDAR_SCAN,
    'down': Topics.Externo.LIDAR_DOWN_SCAN,
    'depth': Topics.Interno.DEPTH_SCAN,
}
_SCAN_KEYS = frozenset(('lidar', 'down', 'depth'))


def _enum_name(value, prefix):
    for name in dir(VehicleStatus):
        if name.startswith(prefix) and not name.endswith('_MAX'):
            if getattr(VehicleStatus, name) == value:
                return name.removeprefix(prefix)
    return f'UNKNOWN({value})'


def _message_values(message):
    """Preserva campos nativos e normaliza mensagens aninhadas recursivamente."""
    if hasattr(message, 'get_fields_and_field_types'):
        return {key: _message_values(value) for key, value in msg_to_dict(message).items()}
    if isinstance(message, dict):
        return {key: _message_values(value) for key, value in message.items()}
    if hasattr(message, 'tolist'):
        return _message_values(message.tolist())
    if isinstance(message, (list, tuple)):
        return [_message_values(value) for value in message]
    return message


def summarize_scan(message):
    """Descarta NaN/Inf e valores fora do intervalo, sem copiar todos os feixes."""
    minimum = math.inf
    valid_count = 0
    bounds_valid = all((
        math.isfinite(message.range_min), math.isfinite(message.range_max),
        0 <= message.range_min <= message.range_max,
    ))
    if bounds_valid:
        for distance in message.ranges:
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                valid_count += 1
                minimum = min(minimum, distance)
    return {
        'minimum_distance': minimum if valid_count else math.nan,
        'beam_count': len(message.ranges),
        'valid_count': valid_count,
        'frame_id': message.header.frame_id,
    }


class DashboardMonitorSubscriber:
    """Recebe cada tópico com sua spec/QoS e atualiza o store sem sinais por frame."""

    def __init__(self, node, store):
        self._store = store
        self.subscriptions = []
        for key, spec in MONITOR_SPECS.items():
            subscription = create_subscription_from(node, spec, partial(self._receive, key))
            self.subscriptions.append(subscription)
            self._store.set_topic_name(key, subscription.topic_name)

    def _receive(self, key, message):
        if key in _SCAN_KEYS:
            values = summarize_scan(message)
        else:
            values = _message_values(message)
            if key == 'status':
                values['nav_state_name'] = _enum_name(message.nav_state, 'NAVIGATION_STATE_')
                values['arming_state_name'] = _enum_name(message.arming_state, 'ARMING_STATE_')
        self._store.update(key, values)
