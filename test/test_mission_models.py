"""Validação de missão e fronteira entre carregar definições e iniciar sessões."""

import json
from pathlib import Path

from drone_inspetor.missions.models import MissionDefinition, MissionValidationError
from drone_inspetor.missions.repository import MissionRepository
from drone_inspetor.nodes.mission_node.config import MissionConfig
from drone_inspetor.nodes.mission_node.session import create_session_directory


import pytest


def definition(point=None):
    """Fornece uma missão no formato público do dashboard."""
    return {'nome': 'Inspeção', 'pontos_de_inspecao': [
        point or {'lat': -22.0, 'lon': -40.0, 'alt': 20.0}]}


def test_repository_roundtrip_is_immutable_and_has_no_session_side_effects(tmp_path):
    source = tmp_path / 'missions.json'
    source.write_text(json.dumps({'teste': definition()}))
    repository = MissionRepository(source)
    missions = repository.load()
    view = repository.as_mapping()
    view['teste']['pontos_de_inspecao'][0]['alt'] = 300
    assert repository.get('teste').waypoints[0].altitude_m == 20
    assert missions['teste'] is repository.get('teste')
    assert list(tmp_path.iterdir()) == [source]
    reconstructed = MissionDefinition.from_mapping('teste', repository.as_mapping()['teste'])
    assert reconstructed == missions['teste']


@pytest.mark.parametrize('point', [
    {'lat': float('nan'), 'lon': 0},
    {'lat': 91, 'lon': 0},
    {'lat': 0, 'lon': 181},
    {'lat': 0, 'lon': 0},
    {'lat': True, 'lon': 0},
    {'lat': 0, 'lon': 0, 'focus_lat': 1},
    {'lat': 0, 'lon': 0, 'use_focus': True},
    {'lat': 0, 'lon': 0, 'ponto_de_deteccao': True},
    {'lat': 0, 'lon': 0, 'ponto_de_deteccao': 'false'},
    {'lat': 0, 'lon': 0, 'tempo_de_permanencia': -1},
    {'lat': 0, 'lon': 0, 'command': 'LAND'},
    {'lat': 0, 'lon': 0, 'ponto_de_deteccao': True, 'objeto_alvo': 'flare',
     'tipos_anomalia': 'Rust'},
])
def test_invalid_waypoint_is_rejected_before_start(point):
    with pytest.raises(MissionValidationError, match='ponto 0'):
        MissionDefinition.from_mapping('teste', definition(point))


def test_bundled_missions_are_valid():
    source = Path(__file__).parents[1] / 'drone_inspetor' / 'missions' / 'missions.json'
    missions = MissionRepository(source).load()
    assert len(missions) == 3
    assert all(mission.waypoints for mission in missions.values())


def test_invalid_reload_preserves_previous_catalog(tmp_path):
    source = tmp_path / 'missions.json'
    source.write_text(json.dumps({'teste': definition()}))
    repository = MissionRepository(source)
    previous = repository.load()
    source.write_text('{"teste": {"pontos_de_inspecao": []}}')
    with pytest.raises(MissionValidationError):
        repository.load()
    assert repository.get('teste') == previous['teste']


def test_session_paths_are_unique_and_created_only_explicitly(tmp_path):
    first = Path(create_session_directory(tmp_path))
    second = Path(create_session_directory(tmp_path))
    assert first != second
    assert (first / 'fotos').is_dir()
    assert (first / 'videos').is_dir()


def test_mission_configuration_resolves_user_paths_and_rejects_bad_deadlines(tmp_path):
    absolute = tmp_path / 'custom.json'
    assert MissionConfig(missions_file=str(absolute)).resolve_missions_file('/share') == absolute
    relative = MissionConfig(missions_file='outro.json').resolve_missions_file('/share')
    assert relative == Path('/share/missions/outro.json')
    with pytest.raises(ValueError):
        MissionConfig(action_feedback_timeout=0)
    with pytest.raises(ValueError):
        MissionConfig(detection_timeout=1, detection_service_timeout=2)


def test_altitude_frame_is_global_for_waypoints_and_relative_for_takeoff():
    mission = MissionDefinition.from_mapping('mar_morto', definition({
        'lat': 31.5, 'lon': 35.5, 'alt': -300.0}))
    assert mission.waypoints[0].altitude_m == -300.0
    assert mission.takeoff_altitude_m == 20.0
