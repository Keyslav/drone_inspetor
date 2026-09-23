"""Contratos ROS reais e interleavings assíncronos sem servidor ou hardware."""

from concurrent.futures import Future
from types import SimpleNamespace

from drone_inspetor.missions.models import InspectionTarget, Waypoint
from drone_inspetor.nodes.mission_node.action_client import (
    ActionStatus, DroneActionClient, build_goal,
)
from drone_inspetor.nodes.mission_node.cv_client import (
    CVClient, anomaly_request, detection_request, recording_request,
)


class Logger:
    """Registra erros sem exigir um nó ROS."""

    def __init__(self):
        self.messages = []

    def info(self, text):
        self.messages.append(text)

    error = warning = info


class Clock:
    """Relógio operacional controlável pelo teste."""

    def __init__(self):
        self.time = 0.0

    def __call__(self):
        return self.time


class FakeAction:
    """Permite entregar aceitação, feedback e resultado fora de ordem."""

    def __init__(self):
        self.sent = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback):
        future = Future()
        self.sent.append((goal, feedback_callback, future))
        return future


class Handle:
    """Goal aceito cujo cancelamento e resultado são independentes."""

    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        future = Future()
        future.set_result(SimpleNamespace(goals_canceling=[self]))
        return future

    def succeed(self):
        result = SimpleNamespace(success=True, message='ok')
        self.result.set_result(SimpleNamespace(status=4, result=result))


def make_action():
    clock = Clock()
    transport = FakeAction()
    client = DroneActionClient(
        transport, Logger(), clock=clock, feedback_timeout=10, cancel_timeout=2)
    return client, transport, clock


def test_real_service_contracts():
    detection = detection_request(InspectionTarget('flare', ('Rust', 'Crack')), 2)
    assert detection.object_name == 'flare'
    assert list(detection.anomaly_types) == ['Rust', 'Crack']
    assert detection.timeout_seconds == 2.0
    assert recording_request(True).start_recording
    assert not hasattr(recording_request(True), 'folder_path')
    assert not anomaly_request(False).enable


def test_goal_units_focus_and_optional_nan():
    import math
    goal = build_goal('GOTO', lat=-22.0, lon=-40.0, alt=30.0)
    assert math.isnan(goal.yaw)
    assert math.isnan(goal.altitude)
    assert not goal.use_focus
    client, transport, _ = make_action()
    client.navigate_to(Waypoint(-22, -40, 30, focus_latitude_deg=-23, focus_longitude_deg=-41))
    goal = transport.sent[0][0]
    assert goal.use_focus and goal.focus_lat == -23.0 and goal.focus_lon == -41.0


def test_one_active_operation_and_terminal_result():
    client, transport, _ = make_action()
    operation = client.arm()
    assert client.takeoff(20) is None
    handle = Handle()
    transport.sent[0][2].set_result(handle)
    handle.succeed()
    assert operation.result.success
    assert not client.busy


def test_cancel_before_acceptance_is_sent_when_handle_arrives():
    client, transport, _ = make_action()
    operation = client.arm()
    assert client.cancel()
    assert client.busy
    handle = Handle()
    transport.sent[0][2].set_result(handle)
    assert handle.cancel_count == 1
    handle.succeed()
    assert operation.result.status is ActionStatus.CANCELED
    assert not client.busy


def test_late_acceptance_after_cancel_timeout_cannot_own_new_operation():
    client, transport, clock = make_action()
    old = client.arm()
    client.cancel()
    clock.time = 3
    client.poll()
    assert old.done
    new = client.takeoff(20)
    old_handle = Handle()
    transport.sent[0][2].set_result(old_handle)
    assert old_handle.cancel_count == 1
    assert client.active is new
    transport.sent[0][1](SimpleNamespace(feedback=None))
    assert new.last_feedback_at == 3
    new_handle = Handle()
    transport.sent[1][2].set_result(new_handle)
    new_handle.succeed()
    assert new.result.success


def test_late_result_after_reset_cannot_complete_new_waypoint():
    client, transport, clock = make_action()
    old = client.navigate_to(Waypoint(0, 0, 10))
    old_handle = Handle()
    transport.sent[0][2].set_result(old_handle)
    client.cancel()
    clock.time = 3
    client.poll()
    new = client.navigate_to(Waypoint(1, 1, 20))
    old_handle.succeed()
    assert old.result.status is ActionStatus.CANCELED
    assert client.active is new
    assert not new.done


def test_timeout_uses_current_operation_feedback_only():
    client, transport, clock = make_action()
    first = client.arm()
    handle = Handle()
    transport.sent[0][2].set_result(handle)
    clock.time = 100
    transport.sent[0][1](SimpleNamespace(feedback=None))
    handle.succeed()
    second = client.takeoff(20)
    clock.time = 111
    client.poll()
    assert second.cancel_requested_at == 111
    clock.time = 114
    client.poll()
    assert second.result.status is ActionStatus.TIMED_OUT
    assert first.result.success


def test_rejected_or_exceptional_goal_finishes_explicitly():
    client, transport, _ = make_action()
    rejected = client.arm()
    transport.sent[0][2].set_result(SimpleNamespace(accepted=False))
    assert rejected.result.status is ActionStatus.REJECTED
    failed = client.arm()
    transport.sent[1][2].set_exception(RuntimeError('transporte caiu'))
    assert failed.result.status is ActionStatus.FAILED
    assert not client.busy


class Service:
    """Captura requests e permite resposta tardia de serviços."""

    def __init__(self):
        self.sent = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.sent.append((request, future))
        return future

    def finish(self, index=0, *, success=True, center=(10.0, 20.0)):
        self.sent[index][1].set_result(SimpleNamespace(
            success=success, message='resposta', bbox_center=center))


def make_cv():
    clock = Clock()
    detection, recording, anomaly = Service(), Service(), Service()
    client = CVClient(detection, recording, anomaly, Logger(), clock=clock,
                      detection_timeout=10, control_timeout=5)
    return client, detection, recording, anomaly, clock


def test_detection_result_is_bound_to_original_point():
    client, detection, _, _, _ = make_cv()
    old = client.request_detection(InspectionTarget('antigo'))
    new = client.request_detection(InspectionTarget('novo'))
    detection.finish(0)
    assert old.done and not old.success
    assert not new.done
    detection.finish(1, center=(50.0, 80.0))
    assert new.success and new.bbox_center == (50, 80)


def test_detection_deadline_discards_late_success():
    client, detection, _, _, clock = make_cv()
    operation = client.request_detection(InspectionTarget('flare'))
    clock.time = 11
    client.poll()
    detection.finish()
    assert operation.done and not operation.success
    assert operation.bbox_center is None


def test_stop_is_dispatched_after_inflight_start_even_when_stop_deadline_passed():
    client, _, recording, _, clock = make_cv()
    start = client.start_recording()
    stop = client.stop_recording()
    assert len(recording.sent) == 1
    clock.time = 7
    client.poll()
    recording.finish(0)
    assert len(recording.sent) == 2
    assert recording.sent[0][0].start_recording
    assert not recording.sent[1][0].start_recording
    recording.finish(1)
    assert start.done and not start.success
    assert stop.done and not stop.success  # O prazo expirado não é reescrito por resposta tardia.


def test_control_failure_and_nonfinite_detection_are_visible():
    client, detection, recording, _, _ = make_cv()
    recording.ready = False
    assert not client.start_recording().success
    operation = client.request_detection(InspectionTarget('flare'))
    detection.finish(center=(float('nan'), 2.0))
    assert operation.done and not operation.success


def test_reset_stop_barrier_is_preserved_before_a_new_session_start():
    client, _, recording, _, _ = make_cv()
    client.start_recording()
    client.stop_recording()
    latest = client.start_recording()
    recording.finish(0)
    assert not recording.sent[1][0].start_recording
    recording.finish(1)
    assert recording.sent[2][0].start_recording
    recording.finish(2)
    assert latest.success


def test_return_retry_waits_for_explicit_rejection_and_accepts_once():
    from drone_inspetor.nodes.mission_node.config import MissionConfig
    from drone_inspetor.nodes.mission_node.fsm.mission.context import MissionFSMContext
    from drone_inspetor.nodes.mission_node.fsm.mission.states.retornando import RetornandoState
    client, transport, clock = make_action()
    runtime = SimpleNamespace(
        drone=SimpleNamespace(is_landed=False, is_armed=True), actions=client,
        monotonic_time=clock, config=MissionConfig(), get_logger=lambda: Logger())
    state = RetornandoState(MissionFSMContext(), runtime)
    state.on_enter()
    state.on_step()
    assert len(transport.sent) == 1
    transport.sent[0][2].set_result(SimpleNamespace(accepted=False))
    clock.time = 0.9
    state.on_step()
    assert len(transport.sent) == 1
    clock.time = 1.0
    state.on_step()
    assert len(transport.sent) == 2
    handle = Handle()
    transport.sent[1][2].set_result(handle)
    clock.time = 50.0
    state.on_step()
    assert len(transport.sent) == 2  # Aceito: orçamento de retry não cancela nem duplica RTL.
    handle.succeed()
    state.on_step()
    assert len(transport.sent) == 2


def test_return_explicit_rejections_stop_at_monotonic_budget():
    from drone_inspetor.nodes.mission_node.config import MissionConfig
    from drone_inspetor.nodes.mission_node.fsm.mission.context import MissionFSMContext
    from drone_inspetor.nodes.mission_node.fsm.mission.states.retornando import RetornandoState
    client, transport, clock = make_action()
    logger = Logger()
    runtime = SimpleNamespace(
        drone=SimpleNamespace(is_landed=False, is_armed=True), actions=client,
        monotonic_time=clock, config=MissionConfig(return_acceptance_timeout=3.0),
        get_logger=lambda: logger)
    context = MissionFSMContext()
    state = RetornandoState(context, runtime)
    state.on_enter()
    for tick in range(8):
        clock.time = float(tick)
        state.on_step()
        for _, _, future in transport.sent:
            if not future.done():
                future.set_result(SimpleNamespace(accepted=False))
    assert len(transport.sent) == 3
    assert 'Prazo de aceitação' in context.failure_reason
    assert len(logger.messages) == 1


def test_return_does_not_retry_unknown_acceptance_or_a_late_accepted_goal():
    from drone_inspetor.nodes.mission_node.config import MissionConfig
    from drone_inspetor.nodes.mission_node.fsm.mission.context import MissionFSMContext
    from drone_inspetor.nodes.mission_node.fsm.mission.states.retornando import RetornandoState
    client, transport, clock = make_action()
    runtime = SimpleNamespace(
        drone=SimpleNamespace(is_landed=False, is_armed=True), actions=client,
        monotonic_time=clock, config=MissionConfig(), get_logger=lambda: Logger())
    state = RetornandoState(MissionFSMContext(), runtime)
    state.on_enter()
    state.on_step()
    clock.time = 11.0
    client.poll()
    clock.time = 14.0
    client.poll()
    state.on_step()
    assert state.operation.result.status is ActionStatus.TIMED_OUT
    assert len(transport.sent) == 1
    handle = Handle()
    transport.sent[0][2].set_result(handle)
    for tick in range(15, 60):
        clock.time = float(tick)
        state.on_step()
    assert handle.cancel_count == 1
    assert len(transport.sent) == 1


def test_return_does_not_retry_failed_execution_or_transport_error():
    from drone_inspetor.nodes.mission_node.config import MissionConfig
    from drone_inspetor.nodes.mission_node.fsm.mission.context import MissionFSMContext
    from drone_inspetor.nodes.mission_node.fsm.mission.states.retornando import RetornandoState
    for accepted in (False, True):
        client, transport, clock = make_action()
        runtime = SimpleNamespace(
            drone=SimpleNamespace(is_landed=False, is_armed=True), actions=client,
            monotonic_time=clock, config=MissionConfig(), get_logger=lambda: Logger())
        state = RetornandoState(MissionFSMContext(), runtime)
        state.on_enter()
        state.on_step()
        if accepted:
            handle = Handle()
            transport.sent[0][2].set_result(handle)
            handle.result.set_result(SimpleNamespace(
                status=6, result=SimpleNamespace(success=False, message='falha de RTL')))
        else:
            transport.sent[0][2].set_exception(RuntimeError('comunicação perdida'))
        clock.time = 2.0
        state.on_step()
        assert len(transport.sent) == 1
        assert state.failure_logged
