"""Publicação real dos callbacks respeita a autoridade dos modos nativos PX4."""

import threading
from types import SimpleNamespace as NS

import pytest
from px4_msgs.msg import VehicleStatus

from drone_inspetor.nodes.drone_node.drone_node import DroneNode
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


def harness(mode, armed):
    setpoints, heartbeats, trajectory_calls = [], [], []
    node = NS(
        _control_lock=threading.RLock(), _setpoint_published=1.,
        telemetry_fresh=lambda: True,
        state_px4=NS(nav_state=mode, is_armed=armed),
        drone_fsm_context=NS(state=DS.EM_VOO, native_command='LAND'),
        deslocamento_fsm=NS(tick=lambda: trajectory_calls.append('fsm')),
        deslocamento_fsm_context=NS(store_static_position=lambda: trajectory_calls.append('hold')),
        trajectory=NS(reset=lambda: trajectory_calls.append('reset'),
                      begin_tick=lambda: trajectory_calls.append('tick'),
                      create_setpoint_for_current_state=lambda: 'setpoint'),
        px4_trajectory_setpoint_pub=NS(publish=setpoints.append),
        px4_offboard_control_mode_pub=NS(publish=heartbeats.append),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=1000000000)),
    )
    return node, setpoints, heartbeats, trajectory_calls


@pytest.mark.parametrize('mode', [VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
                                 VehicleStatus.NAVIGATION_STATE_AUTO_RTL,
                                 VehicleStatus.NAVIGATION_STATE_AUTO_MISSION,
                                 VehicleStatus.NAVIGATION_STATE_POSCTL])
def test_armed_native_modes_receive_no_competing_ros_setpoints(mode):
    node, setpoints, heartbeats, trajectory_calls = harness(mode, armed=True)
    DroneNode.tick_deslocamento_e_publish_setpoint(node)
    DroneNode.px4_publish_offboard_control_mode(node)
    assert not setpoints and not heartbeats and not trajectory_calls
    assert node._setpoint_published is None


def test_pending_handover_keeps_stream_until_px4_leaves_offboard():
    node, setpoints, heartbeats, trajectory_calls = harness(
        VehicleStatus.NAVIGATION_STATE_OFFBOARD, armed=True)
    DroneNode.tick_deslocamento_e_publish_setpoint(node)
    DroneNode.px4_publish_offboard_control_mode(node)
    assert setpoints == ['setpoint'] and len(heartbeats) == 1
    assert trajectory_calls == ['tick', 'fsm']


def test_disarmed_vehicle_can_prime_offboard_again_after_native_landing():
    node, setpoints, heartbeats, trajectory_calls = harness(
        VehicleStatus.NAVIGATION_STATE_AUTO_LAND, armed=False)
    DroneNode.tick_deslocamento_e_publish_setpoint(node)
    DroneNode.px4_publish_offboard_control_mode(node)
    assert setpoints == ['setpoint'] and len(heartbeats) == 1
    assert trajectory_calls == ['reset', 'hold', 'tick']
