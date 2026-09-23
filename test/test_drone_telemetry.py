"""Contrato ROS preserva a convenção legada de Z sem contaminar a navegação."""

import math
from types import SimpleNamespace as NS

from drone_inspetor.nodes.drone_node.telemetry import drone_state_message
from drone_inspetor.nodes.drone_node.px4_state import DroneStatePX4
from drone_inspetor.nodes.drone_node.target_stack import TargetStack
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription as TS


def test_position_z_is_up_while_derivatives_are_ned():
    state = DroneStatePX4()
    state.local_position = NS(x=1., y=2., z=-3.)
    state.current_velocity_z = -.5
    state.current_acceleration_z = -.2
    state.current_yaw_deg_normalized = -90.
    state.current_yaw_rad = -math.pi / 2
    maneuver = NS(state=TS.DESLOCANDO, target_stack=TargetStack(),
                  last_static_position=[0., 0., -2.], last_static_yaw_deg=None,
                  last_static_yaw_deg_normalized=None, last_static_yaw_rad=None)
    maneuver.target_stack.push_missao([10., 2., -5.])
    lifecycle = NS(state=DS.EM_VOO, state_entry_time=2.)
    message = drone_state_message(state, lifecycle, maneuver, 4.)
    assert message.current_local_z == 3.
    assert message.target_local_z == 5.
    assert message.last_static_position_z == 2.
    assert message.current_velocity_z == -.5
    assert message.current_acceleration_z == -.2
    assert message.current_yaw_deg == 270.
    assert message.current_yaw_deg_normalized == -90.
    assert message.state_duration_sec == 2.
    assert math.isnan(message.home_local_z)
    assert math.isnan(message.focus_lat)
    assert state.local_position.z == -3.
