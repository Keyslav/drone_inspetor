"""Regressões de concorrência, conclusão física e transferência nativa PX4."""

from concurrent.futures import ThreadPoolExecutor
from threading import Barrier, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from rclpy.action import CancelResponse, GoalResponse

from drone_inspetor_msgs.action import DroneCommand
from drone_inspetor.nodes.drone_node import action_server
from drone_inspetor.nodes.drone_node.action_server import DroneActionServerMixin
from drone_inspetor.nodes.drone_node.px4_commands import DronePX4CommandsMixin
from drone_inspetor.nodes.drone_node.px4_state import DroneStatePX4
from drone_inspetor.nodes.drone_node.fsm.drone.context import DroneFSMContext
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.drone_node.fsm.drone.machine import DroneFSM
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import (
    DeslocamentoFSMDescription as TS,
)
from drone_inspetor.nodes.drone_node.target_stack import TargetStack
from drone_inspetor.nodes.drone_node.trajectory import Trajectory


class Harness(DroneActionServerMixin, DronePX4CommandsMixin):
    """Usa contratos/contexto/FSM reais, com transporte e dinâmica substituídos."""

    def __init__(self):
        self.param_px4_target_system_id = 1
        self._control_lock = RLock()
        self._is_drone_node_shutting_down = False
        self.logger = Mock()
        self.fresh = True
        self.state_px4 = DroneStatePX4()
        px4 = self.state_px4
        px4.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        px4.is_armed = True
        px4.is_landed = False
        px4.local_position = SimpleNamespace(x=10., y=20., z=-2.)
        px4.global_position = SimpleNamespace(lat=-22., lon=-43., alt=104.)
        px4.home_global_lat, px4.home_global_lon, px4.home_global_alt = -22., -43., 100.
        px4.home_local_position = [10., 20., 2.]
        self.trajectory = SimpleNamespace(
            stopped=True, reference_stopped=True, navigation_error=None, begin_command=self.begin_command,
            reset=Mock(), node=self,
        )
        def convert(*args):
            return Trajectory.global_to_local_position(self.trajectory, *args)

        self.trajectory.global_to_local_position = convert
        self.drone_fsm_context = DroneFSMContext(self)
        stack = TargetStack()
        self.deslocamento_fsm_context = SimpleNamespace(
            target_stack=stack, state=TS.PLANANDO, initial_distance_to_target=None,
            store_static_position=Mock(), reset=lambda: stack.clear(),
        )
        self.deslocamento_fsm = SimpleNamespace(reset_to_planando=self.reset_to_planando)
        self.drone_fsm = DroneFSM(self.drone_fsm_context, self)
        self.drone_fsm.register_all_states()
        self.drone_fsm.transition_to(DS.EM_VOO)
        self.drone_fsm.tick()
        self.px4_vehicle_command_pub = Mock()
        self.init_px4_commands_state()
        self.init_action_server_state()

    def get_logger(self):
        return self.logger

    def get_clock(self):
        # Relógio ROS pausado; timeouts ainda devem funcionar.
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))

    def telemetry_fresh(self):
        return self.fresh

    def begin_command(self):
        self.trajectory.navigation_error = None

    def reset_to_planando(self):
        self.deslocamento_fsm_context.state = TS.PLANANDO


class Goal:
    def __init__(self, request):
        self.request = request
        self.is_cancel_requested = False
        self.terminal = []
        self.feedback = []

    def succeed(self):
        self.terminal.append('success')

    def abort(self):
        self.terminal.append('abort')

    def canceled(self):
        self.terminal.append('canceled')

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)


@pytest.fixture
def clock(monkeypatch):
    clock = SimpleNamespace(now=0., callback=lambda: None)

    def sleep(seconds):
        clock.now += seconds
        assert clock.now < 10., 'Loop de action não terminou no cenário de teste'
        clock.callback()

    monkeypatch.setattr(action_server, 'time', SimpleNamespace(
        monotonic=lambda: clock.now, sleep=sleep,
    ))
    return clock


def request(command='GOTO', **kwargs):
    return DroneCommand.Goal(
        command=command, lat=float('nan'), lon=float('nan'), alt=float('nan'),
        yaw=float('nan'), **kwargs,
    )


def accept(node, payload):
    assert node.goal_callback(payload) == GoalResponse.ACCEPT
    return Goal(payload)


def test_reservation_rejects_parallel_goals_before_execution():
    node = Harness()
    barrier = Barrier(8)

    def reserve(_):
        barrier.wait()
        return node.goal_callback(request())

    with ThreadPoolExecutor(max_workers=8) as executor:
        results = list(executor.map(reserve, range(8)))
    assert results.count(GoalResponse.ACCEPT) == 1
    assert results.count(GoalResponse.REJECT) == 7


@pytest.mark.parametrize('command', ['unknown', '', 'GOTO_FOCUS'])
def test_unknown_command_rejected(command):
    node = Harness()
    assert node.goal_callback(request(command)) == GoalResponse.REJECT
    assert not node._is_command_complete(command)


@pytest.mark.parametrize('field,value', [
    ('lat', 91.), ('lon', -181.), ('alt', float('inf')), ('yaw', float('-inf')),
    ('focus_lat', float('inf')),
])
def test_invalid_goto_reference_does_not_reserve(field, value):
    node = Harness()
    payload = request()
    setattr(payload, field, value)
    assert node.goal_callback(payload) == GoalResponse.REJECT
    assert node._active_command is None


def test_cancel_before_execute_does_not_dispatch(clock):
    node = Harness()
    goal = accept(node, request())
    assert node.cancel_callback(goal) == CancelResponse.ACCEPT
    goal.is_cancel_requested = True
    result = node.execute_drone_command_callback(goal)
    assert not result.success
    assert goal.terminal == ['canceled']
    assert node.deslocamento_fsm_context.target_stack.is_empty
    assert node._active_command is None


def test_timeout_uses_monotonic_clock_and_stops_before_release(clock):
    node = Harness()
    goal = accept(node, request())
    node.command_timeouts['GOTO'] = 0.1
    stopped_at = []

    def advance():
        node.trajectory.stopped = clock.now >= 0.4
        if node.deslocamento_fsm_context.target_stack.is_empty:
            stopped_at.append(clock.now)
        if clock.now < 0.4:
            assert node._active_command is not None

    clock.callback = advance
    result = node.execute_drone_command_callback(goal)
    assert not result.success and 'Timeout' in result.message
    assert goal.terminal == ['abort']
    assert stopped_at and clock.now >= 0.4
    assert node.deslocamento_fsm_context.target_stack.is_empty
    assert node._active_command is None


def test_cancel_has_priority_when_arrival_occurs_same_cycle(clock):
    node = Harness()
    goal = accept(node, request())

    def cancel_at_arrival():
        node.deslocamento_fsm_context.target_stack.clear()
        assert node.cancel_callback(goal) == CancelResponse.ACCEPT
        goal.is_cancel_requested = True

    clock.callback = cancel_at_arrival
    result = node.execute_drone_command_callback(goal)
    assert not result.success and goal.terminal == ['canceled']


def test_navigation_error_cannot_turn_empty_stack_into_success(clock):
    node = Harness()
    goal = accept(node, request())

    def fault():
        node.deslocamento_fsm_context.target_stack.clear()
        node.trajectory.navigation_error = 'obstáculo sem desvio'

    clock.callback = fault
    result = node.execute_drone_command_callback(goal)
    assert not result.success and 'obstáculo' in result.message
    assert goal.terminal == ['abort']


def test_missing_home_fails_without_queueing_target(clock):
    node = Harness()
    payload = request()
    payload.lat, payload.lon, payload.alt = -22., -43., 105.
    goal = accept(node, payload)
    node.state_px4.home_global_lat = None
    result = node.execute_drone_command_callback(goal)
    assert not result.success and 'HOME' in result.message
    assert node.deslocamento_fsm_context.target_stack.is_empty
    assert node._active_command is None


def test_goto_respects_home_offset_and_partial_coordinates():
    node = Harness()
    node.goto(lat=float('nan'), lon=-43., alt=110., use_focus=True,
              focus_lat=-22., focus_lon=-43.)
    target = node.deslocamento_fsm_context.target_stack.current
    assert target.local_position == [10., 20., -8.]
    assert target.focus_local_position == [10., 20., 2.]
    assert target.latitude == -22.


def test_failed_goto_preserves_stack_atomically():
    node = Harness()
    node.goto()
    original = node.deslocamento_fsm_context.target_stack.current
    with pytest.raises(ValueError):
        node.goto(lat=92., lon=-43., alt=110.)
    assert node.deslocamento_fsm_context.target_stack.current is original


def test_cancel_takeoff_switches_to_hover_without_resetting_profile(clock):
    node = Harness()
    node.drone_fsm.transition_to(DS.DECOLANDO)
    node.stop()
    assert node.drone_fsm_context.state == DS.EM_VOO
    assert node.deslocamento_fsm_context.state == TS.PLANANDO
    node.trajectory.reset.assert_not_called()


@pytest.mark.parametrize('command,mode', [
    ('LAND', VehicleStatus.NAVIGATION_STATE_AUTO_LAND),
    ('RTL', VehicleStatus.NAVIGATION_STATE_AUTO_RTL),
])
def test_native_handover_survives_mode_change_and_requires_landing(clock, command, mode):
    node = Harness()
    goal = accept(node, request(command))

    def native_progress():
        node.state_px4.nav_state = mode
        node.drone_fsm.tick()
        assert node.cancel_callback(goal) == CancelResponse.REJECT
        if clock.now < 0.1:
            assert node.drone_fsm_context.state == DS.EM_VOO
            assert not node._is_command_complete(command)
        else:
            node.state_px4.is_landed = True
            node.state_px4.is_armed = command == 'LAND'
            node.drone_fsm.tick()

    clock.callback = native_progress
    result = node.execute_drone_command_callback(goal)
    assert result.success and goal.terminal == ['success']
    node.drone_fsm.tick()
    assert node.drone_fsm_context.state in (DS.POUSADO_ARMADO, DS.POUSADO_DESARMADO)


def test_unexpected_offboard_loss_remains_failure(clock):
    node = Harness()
    goal = accept(node, request())

    def manual_mode():
        node.state_px4.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
        node.drone_fsm.tick()

    clock.callback = manual_mode
    result = node.execute_drone_command_callback(goal)
    assert not result.success and 'OFFBOARD' in result.message


def test_ack_rejection_aborts_instead_of_waiting_for_success(clock):
    node = Harness()
    goal = accept(node, request('LAND'))

    def rejected():
        node.px4_command_ack_callback(VehicleCommandAck(
            command=VehicleCommand.VEHICLE_CMD_NAV_LAND,
            result=VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
        ))

    clock.callback = rejected
    result = node.execute_drone_command_callback(goal)
    assert not result.success and 'PX4 rejeitou' in result.message


def test_disarm_rejected_while_in_air():
    node = Harness()
    assert node.goal_callback(request('DISARM')) == GoalResponse.REJECT
    with pytest.raises(ValueError):
        node.disarm_drone()


def test_takeoff_accepts_ground_velocity_noise_but_waits_before_starting():
    node = Harness()
    node.state_px4.is_landed = True
    node.drone_fsm.transition_to(DS.POUSADO_ARMADO)
    node.trajectory.stopped = False
    payload = request('TAKEOFF', altitude=3.)
    accept(node, payload)
    node._execute_command(payload)
    for _ in range(3):
        node.drone_fsm.tick()
        assert node.drone_fsm_context.state == DS.POUSADO_ARMADO
        assert node.drone_fsm_context.pending_command == 'TAKEOFF'
    node.state_px4.is_landed = False
    node.drone_fsm.tick()
    assert node.drone_fsm_context.state == DS.POUSADO_ARMADO
    assert not node._is_command_complete('TAKEOFF')
    node.state_px4.is_landed = True
    node.trajectory.stopped = True
    node.drone_fsm.tick()
    assert node.drone_fsm_context.state == DS.DECOLANDO
    assert node.drone_fsm_context.pending_command is None


def test_new_move_is_rejected_while_previous_reference_is_braking():
    node = Harness()
    node.trajectory.reference_stopped = False
    assert node.goal_callback(request('GOTO')) == GoalResponse.REJECT
    assert node._active_command is None


def test_stop_is_a_supported_goal_and_waits_for_braking(clock):
    node = Harness()
    goal = accept(node, request('STOP'))
    node.trajectory.stopped = False
    clock.callback = lambda: setattr(node.trajectory, 'stopped', clock.now >= 0.1)
    result = node.execute_drone_command_callback(goal)
    assert result.success and clock.now >= 0.1


def test_cancel_waits_for_rclpy_canceling_transition_before_finishing(clock):
    node = Harness()
    goal = accept(node, request())
    assert node.cancel_callback(goal) == CancelResponse.ACCEPT

    def mark_canceling():
        goal.is_cancel_requested = True

    clock.callback = mark_canceling
    result = node.execute_drone_command_callback(goal)
    assert not result.success and goal.terminal == ['canceled']
    assert clock.now > 0
    assert node.deslocamento_fsm_context.target_stack.is_empty


def test_native_rejection_allows_retry_while_still_offboard(clock):
    node = Harness()
    goal = accept(node, request('LAND'))

    def rejected():
        node.px4_command_ack_callback(VehicleCommandAck(
            command=VehicleCommand.VEHICLE_CMD_NAV_LAND,
            result=VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
        ))

    clock.callback = rejected
    assert not node.execute_drone_command_callback(goal).success
    assert node.drone_fsm_context.native_command is None
    assert node.goal_callback(request('LAND')) == GoalResponse.ACCEPT


def test_native_timeout_does_not_take_control_from_px4(clock):
    node = Harness()
    node.command_timeouts['LAND'] = 0.1
    goal = accept(node, request('LAND'))

    def native_mode():
        node.state_px4.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        node.drone_fsm.tick()

    clock.callback = native_mode
    assert not node.execute_drone_command_callback(goal).success
    assert node.drone_fsm_context.native_command == 'LAND'
    assert node.drone_fsm_context.state == DS.EM_VOO
    assert node.goal_callback(request()) == GoalResponse.REJECT
