"""Mapeia snapshots internos NED para o contrato legado DroneStateMSG.

Posições Z da mensagem são positivas para cima; velocidade e aceleração Z
permanecem NED (positivas para baixo). Essa diferença é preservada por
compatibilidade e fica concentrada nesta borda, não nos cálculos de navegação.
"""

import math

from drone_inspetor_msgs.msg import DroneStateMSG
from drone_inspetor.common.coordinates import ned_to_legacy_neu
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import (
    DeslocamentoFSMDescription as TS,
)


def _number(value):
    return math.nan if value is None else float(value)


def _position(message, prefix, position, missing=math.nan):
    values = ((missing, missing, missing) if position is None
              else ned_to_legacy_neu(tuple(float(value) for value in position)))
    for axis, value in zip('xyz', values):
        setattr(message, f'{prefix}_{axis}', value)


def drone_state_message(px4, lifecycle, maneuver, now):
    """Produz uma mensagem nova sem alterar estado nem acessar transporte ROS."""
    message = DroneStateMSG()
    message.state = int(lifecycle.state)
    message.state_name = lifecycle.state.name
    message.state_duration_sec = round(max(0., now - lifecycle.state_entry_time), 2)
    message.is_armed = px4.is_armed
    message.is_landed = px4.is_landed
    message.is_on_trajectory = maneuver.state != TS.PLANANDO
    current = px4.local_position
    _position(message, 'current_local',
              None if current is None else (current.x, current.y, current.z), missing=0.)
    global_position = px4.global_position
    for field, attribute in (('latitude', 'lat'), ('longitude', 'lon'), ('altitude', 'alt')):
        setattr(message, f'current_{field}',
                0. if global_position is None else float(getattr(global_position, attribute)))
    message.current_yaw_deg_normalized = px4.current_yaw_deg_normalized
    message.current_yaw_deg = px4.current_yaw_deg_normalized % 360.
    message.current_yaw_rad = px4.current_yaw_rad
    for field in ('lat', 'lon', 'alt'):
        setattr(message, f'home_global_{field}', _number(getattr(px4, f'home_global_{field}')))
    _position(message, 'home_local', px4.home_local_position)
    for field in ('deg', 'deg_normalized', 'rad'):
        setattr(message, f'home_yaw_{field}', _number(getattr(px4, f'home_yaw_{field}')))

    target = maneuver.target_stack.current
    _position(message, 'target_local', None if target is None else target.local_position)
    for field, attribute in (('lat', 'latitude'), ('lon', 'longitude'), ('alt', 'altitude')):
        setattr(message, f'target_{field}',
                _number(None if target is None else getattr(target, attribute)))
    for group in ('direction', 'final'):
        for unit in ('deg', 'deg_normalized', 'rad'):
            field = f'{group}_yaw_{unit}'
            setattr(message, f'target_{field}',
                    _number(None if target is None else getattr(target, field)))
    message.focus_lat = _number(None if target is None else target.focus_latitude)
    message.focus_lon = _number(None if target is None else target.focus_longitude)
    message.focus_yaw_deg = message.focus_yaw_deg_normalized = message.focus_yaw_rad = math.nan
    _position(message, 'last_static_position', maneuver.last_static_position)
    for unit in ('deg', 'deg_normalized', 'rad'):
        field = f'last_static_yaw_{unit}'
        setattr(message, field, _number(getattr(maneuver, field)))
    message.has_trajectory_adjusted = target is not None and target.is_desvio
    _position(message, 'trajectory_adjusted',
              target.local_position if message.has_trajectory_adjusted else None)
    for quantity in ('velocity', 'acceleration'):
        for axis in 'xyz':
            field = f'current_{quantity}_{axis}'
            setattr(message, field, float(getattr(px4, field)))
    return message
