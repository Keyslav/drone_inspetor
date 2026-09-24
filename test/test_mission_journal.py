"""Eventos persistem entre falhas, resets e sessões distintas."""

import json
from drone_inspetor.nodes.mission_node.journal import MissionJournal


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_changes_are_immediate_and_snapshots_limited_even_when_ros_pauses(tmp_path):
    now = [0.0]
    errors = []
    journal = MissionJournal(errors.append, clock=lambda: now[0])
    journal.start(tmp_path, {'name': 'Flare'}, {'takeoff_altitude': 20}, 0.)
    mission = dict(state_name='EXECUTANDO_DECOLANDO', ponto_de_inspecao_indice_atual=0)
    drone = dict(state_name='DECOLANDO')
    journal.observe(0., mission, drone, '', 0.1)
    now[0] = 0.2
    journal.observe(0., mission, drone, '', 0.1)
    now[0] = 1.1
    journal.observe(0., mission, drone, '', 0.1)
    now[0] = 1.2
    mission['state_name'] = 'RETORNANDO'
    journal.observe(0., mission, drone, 'Telemetria local expirada', 0.6)
    data = rows(tmp_path / 'events.jsonl')  # legível antes de fechar
    assert [r['event'] for r in data] == ['session_start', 'state_change', 'snapshot', 'state_change']
    assert data[-1]['failure_reason'] == 'Telemetria local expirada'
    assert data[-1]['elapsed_s'] == 1.2 and data[-1]['ros_time_s'] == 0.
    assert not errors
    journal.close()


def test_new_session_is_separate_and_records_command_result(tmp_path):
    first, second = tmp_path / 'first', tmp_path / 'second'
    first.mkdir()
    second.mkdir()
    journal = MissionJournal(lambda error: None)
    journal.start(first, {}, {}, 0.)
    journal.record('rosout', 1., node='mission_node', message='TAKEOFF: falhou')
    journal.start(second, {}, {}, 2.)
    journal.close()
    assert len(rows(first / 'events.jsonl')) == 2
    assert len(rows(second / 'events.jsonl')) == 1


def test_unwritable_journal_does_not_interrupt_control(tmp_path):
    errors = []
    journal = MissionJournal(errors.append)
    journal.start(tmp_path / 'missing', {}, {}, 0.)
    journal.record('sample', 1.)
    assert len(errors) == 1
