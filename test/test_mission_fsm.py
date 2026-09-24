"""Sequências da FSM, sem PX4, serviços externos ou nós de operação."""

from types import SimpleNamespace

from drone_inspetor.missions.models import MissionDefinition
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.mission_node.action_client import (
    ActionResult, ActionStatus, FlightOperation,
)
from drone_inspetor.nodes.mission_node.config import MissionConfig
from drone_inspetor.nodes.mission_node.cv_client import CVOperation
from drone_inspetor.nodes.mission_node.fsm.mission.context import MissionFSMContext
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS
from drone_inspetor.nodes.mission_node.fsm.mission.machine import MissionFSM
from drone_inspetor.nodes.mission_node.runtime import MissionRuntime


class Logger:
    """Captura logs para assegurar que falhas são observáveis."""

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    error = warning = info


class Actions:
    """Resultados controlados sem inferir chegada a partir de estado global."""

    def __init__(self):
        self.sent = []
        self.busy = False
        self.pending = None
        self.automatic = True

    def _send(self, command, payload=None):
        if self.busy:
            return None
        operation = FlightOperation(len(self.sent) + 1, command, 0, 0)
        self.sent.append((operation, payload))
        if self.automatic:
            operation.result = ActionResult(ActionStatus.SUCCEEDED, 'ok')
        else:
            self.busy = True
            self.pending = operation
        return operation

    def arm(self):
        return self._send('ARM')

    def takeoff(self, altitude_m):
        return self._send('TAKEOFF', altitude_m)

    def navigate_to(self, waypoint):
        return self._send('GOTO', waypoint)

    def return_home(self):
        return self._send('RTL')

    def cancel(self):
        if self.pending is not None:
            self.pending.result = ActionResult(ActionStatus.CANCELED, 'cancelado')
            self.pending = None
        self.busy = False

    def poll(self):
        pass


class Vision:
    """Simula confirmação de gravação, anomalias e detecção."""

    def __init__(self):
        self.calls = []
        self.result = True
        self.record_result = True

    def _result(self, command, success=True):
        self.calls.append(command)
        operation = CVOperation(len(self.calls), 0, 10)
        operation.finish(success, 'ok' if success else 'erro', (10, 20) if success else None)
        return operation

    def request_detection(self, target):
        return self._result(('detect', target.object_name), self.result)

    def cancel_detection(self):
        pass

    def start_recording(self):
        return self._result('start', self.record_result)

    def enable_anomalies(self, enabled):
        return self._result(('anomaly', enabled))

    def stop_inspection(self):
        return self._result('stop'), self._result(('anomaly', False))

    def poll(self):
        pass


def fixture():
    """Monta FSM com o mesmo runtime utilizado pelo nó real."""
    time = SimpleNamespace(ros=0.0, healthy=True)
    drone = SimpleNamespace(state=DS.POUSADO_DESARMADO, is_landed=True, is_armed=False)
    runtime = MissionRuntime(drone, Actions(), Vision(), MissionConfig(), Logger(),
                             lambda: time.ros, lambda: time.healthy)
    context = MissionFSMContext()
    machine = MissionFSM(context, runtime)
    machine.register_all_states()
    machine.transition_to(MS.DESATIVADO)
    return machine, context, runtime, time


def mission(with_detection=True):
    """Dois pontos para verificar correlação da chegada e avanço."""
    return MissionDefinition.from_mapping('teste', {
        'nome': 'teste', 'tempo_de_permanencia': 2.0,
        'pontos_de_inspecao': [
            {'lat': 0.0, 'lon': 0.0, 'alt': 20.0, 'ponto_de_deteccao': with_detection,
             'objeto_alvo': 'flare', 'tipos_anomalia': ['Rust']},
            {'lat': 1.0, 'lon': 1.0, 'alt': 30.0},
        ],
    })


def reach_inspection(machine, context, runtime):
    """Conduz prontidão, ARM e TAKEOFF até o primeiro ponto."""
    machine.tick()
    assert machine.current_state_id == MS.PRONTO
    context.start(mission(), '/tmp/sessao-teste')
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_ARMANDO
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_DECOLANDO
    runtime.drone.state = DS.EM_VOO
    runtime.drone.is_armed = True
    runtime.drone.is_landed = False
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO


def test_complete_mission_waits_for_scan_and_returns_without_repeating_rtl():
    machine, context, runtime, time = fixture()
    reach_inspection(machine, context, runtime)
    machine.tick()
    assert context.waypoint_reached  # Tempo ROS zero não significa "ainda não chegou".
    assert context.ponto_de_inspecao_tempo_de_chegada == 0
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO_DETECTANDO
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO_ESCANEANDO
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO_ESCANEANDO
    time.ros = 2.0
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO
    machine.tick()
    assert context.ponto_de_inspecao_indice_atual == 1
    assert not context.waypoint_reached
    machine.tick()
    assert context.ponto_de_inspecao_indice_atual == 2
    machine.tick()
    assert machine.current_state_id == MS.INSPECAO_FINALIZADA
    machine.tick()
    assert machine.current_state_id == MS.RETORNANDO
    machine.tick()
    machine.tick()
    assert [operation.command for operation, _ in runtime.actions.sent] == [
        'ARM', 'TAKEOFF', 'GOTO', 'GOTO', 'RTL']
    runtime.drone.is_landed = True
    runtime.drone.is_armed = False
    runtime.drone.state = DS.OFFBOARD_DESATIVADO
    machine.tick()
    assert machine.current_state_id == MS.DESATIVADO
    assert context.mission is None
    assert not context.on_mission


def test_telemetry_loss_resets_authoritative_state_and_invalidates_operation():
    machine, context, runtime, time = fixture()
    reach_inspection(machine, context, runtime)
    runtime.actions.automatic = False
    machine.tick()
    old = runtime.actions.pending
    time.healthy = False
    machine.tick()
    assert machine.current_state_id == MS.DESATIVADO
    assert machine.current_state.__class__.__name__ == 'DesativadoState'
    assert context.mission is None
    assert not context.waypoint_reached
    assert old.result.status is ActionStatus.CANCELED


def test_cancel_during_navigation_cannot_mark_arrival_and_sends_rtl():
    machine, context, runtime, _ = fixture()
    reach_inspection(machine, context, runtime)
    runtime.actions.automatic = False
    machine.tick()
    old = runtime.actions.pending
    context.cancel_mission = True
    machine.tick()
    assert machine.current_state_id == MS.RETORNANDO
    assert not context.waypoint_reached
    assert old.result.status is ActionStatus.CANCELED
    machine.tick()
    assert runtime.actions.sent[-1][0].command == 'RTL'
    assert context.ponto_de_inspecao_indice_atual == 0


def test_detection_failure_skips_exactly_one_point():
    machine, context, runtime, _ = fixture()
    reach_inspection(machine, context, runtime)
    runtime.cv.result = False
    machine.tick()
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO
    assert context.ponto_de_inspecao_indice_atual == 1
    machine.tick()
    assert runtime.actions.sent[-1][1].latitude_deg == 1.0


def test_recording_failure_ends_inspection_instead_of_claiming_success():
    machine, context, runtime, _ = fixture()
    reach_inspection(machine, context, runtime)
    runtime.cv.record_result = False
    machine.tick()
    machine.tick()
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO_FALHA
    assert context.ponto_de_inspecao_indice_atual == 0
    machine.tick()
    assert machine.current_state_id == MS.RETORNANDO
    assert context.failure_reason == 'erro'


def test_reset_is_idempotent_and_state_changes_only_in_machine():
    machine, context, runtime, _ = fixture()
    reach_inspection(machine, context, runtime)
    machine.reset('teste')
    machine.reset('teste repetido')
    assert machine.current_state_id == MS.DESATIVADO
    assert not hasattr(context, 'state')
    assert context.mission is None
    assert not runtime.actions.busy


def test_ros_clock_jump_restarts_scan_duration():
    machine, context, runtime, time = fixture()
    reach_inspection(machine, context, runtime)
    machine.tick()
    machine.tick()
    time.ros = 100
    machine.tick()
    time.ros = 1
    machine.tick()
    time.ros = 2
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO_ESCANEANDO
    time.ros = 3
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO


def test_message_state_comes_from_machine_after_health_reset():
    from drone_inspetor.nodes.mission_node.mission_node import MissionNode
    machine, context, runtime, time = fixture()
    reach_inspection(machine, context, runtime)
    time.healthy = False
    machine.tick()
    published = []
    adapter = SimpleNamespace(mission_ctx=context, mission_machine=machine,
                              _journal_session_active=False,
                              mission_state_pub=SimpleNamespace(publish=published.append))
    MissionNode.publish_mission_state(adapter)
    assert published[0].state == int(machine.current_state_id) == int(MS.DESATIVADO)
    assert published[0].state_name == machine.current_state_id.name
    assert not published[0].on_mission


def test_takeoff_stale_telemetry_returns_without_sending_first_waypoint():
    machine, context, runtime, _ = fixture()
    machine.tick()
    context.start(mission(), '/tmp/sessao-teste')
    machine.tick()
    machine.tick()
    assert machine.current_state_id == MS.EXECUTANDO_DECOLANDO
    runtime.actions.automatic = False
    machine.tick()
    runtime.actions.pending.result = ActionResult(ActionStatus.FAILED, 'Telemetria local expirada')
    runtime.actions.busy = False
    runtime.drone.state = DS.EM_VOO
    runtime.drone.is_landed = False
    runtime.drone.is_armed = True
    machine.tick()
    assert machine.current_state_id == MS.RETORNANDO
    assert context.failure_reason == 'Telemetria local expirada'
    machine.tick()
    assert [operation.command for operation, _ in runtime.actions.sent] == ['ARM', 'TAKEOFF', 'RTL']
    assert any('Telemetria local expirada' in message for message in runtime.logger.messages)
