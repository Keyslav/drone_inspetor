"""Validação móvel sem ROS: validade, idempotência e fronteira de arquivos."""

import json
import math

from drone_inspetor.mobile_gateway.api import GatewayError, MobileAPI
from drone_inspetor.mobile_gateway.state import MobileState

import pytest


class FakeAdapter:
    """Registra efeitos solicitados e pode simular resposta perdida."""

    def __init__(self):
        """Mantém o histórico por instância."""
        self.calls = []
        self.timeout = False

    def execute(self, command, args):
        """Registra antes do timeout, como um serviço que recebeu o pedido."""
        self.calls.append((command, dict(args)))
        if self.timeout:
            raise TimeoutError('resposta perdida')
        return {'status': 'submitted', 'message': 'Pedido encaminhado.'}

    def models(self):
        """Fornece modelos disponíveis e indisponíveis no contrato ROS."""
        return {'models': [
            {'file_name': 'equipment.pt', 'object_type': 'equipment', 'available': True},
            {'file_name': 'anomaly.pt', 'object_type': 'anomaly', 'available': True},
            {'file_name': 'missing.pt', 'object_type': 'equipment', 'available': False},
        ]}


@pytest.fixture
def gateway():
    """Constrói uma API habilitada com relógio controlável e estado recente."""
    now = [100.0]
    state = MobileState(clock=lambda: now[0])
    state.update('mission', {'state_name': 'PRONTO', 'on_mission': False})
    state.update('drone', {'state_name': 'PRONTO'})
    state.update('status', {'nav_state_name': 'OFFBOARD'})
    adapter = FakeAdapter()
    api = MobileAPI(state, adapter, {'Flare': {}}, enable_commands=True, clock=lambda: now[0])
    return state, adapter, api, now


def _body(api, command='mission.start', args=None, **overrides):
    request = {'id': 'request_0001', 'command': command,
               'args': {'mission': 'Flare'} if args is None else args,
               'confirmed': True, 'nonce': api.snapshot()['command_nonce']}
    request.update(overrides)
    return request


def test_state_expires_frames_and_preserves_unknown_sensor_values(gateway):
    """Imagem antiga desaparece; ausência numérica sai como null em JSON estrito."""
    state, _adapter, _api, now = gateway
    state.update('depth', {'minimum_distance': math.nan})
    state.update_extra('global', {'alt': math.inf, 'nested': [math.nan]})
    state.update_radar([(2.0, 0.1), (math.nan, 0), (-1, 0), (2, math.inf)])
    state.put_frame('camera', b'jpeg-test')
    snapshot = state.snapshot()
    json.dumps(snapshot, allow_nan=False)
    assert snapshot['topics']['depth']['values']['minimum_distance'] is None
    assert snapshot['topics']['global']['values'] == {'alt': None, 'nested': [None]}
    assert snapshot['radar']['points'] == [[2.0, 0.1]]
    assert state.frame('camera') == b'jpeg-test'
    now[0] += 3.1
    assert state.frame('camera') is None
    assert state.snapshot()['topics']['global']['health'] == 'stale'


@pytest.mark.parametrize('stale_key', ['mission', 'drone', 'status'])
def test_mission_commands_reject_each_stale_dependency(gateway, stale_key):
    """Um único produtor expirado impede efeito de missão."""
    state, adapter, api, now = gateway
    now[0] += 4
    for key in ('mission', 'drone', 'status'):
        if key != stale_key:
            state.update(key, {'state_name': 'PRONTO', 'on_mission': False})
    with pytest.raises(GatewayError, match='desatualizada') as error:
        api.execute(_body(api))
    assert error.value.status == 409
    assert adapter.calls == []


@pytest.mark.parametrize('confirmed', [False, None, 1, 'true'])
def test_mission_commands_require_literal_confirmation(gateway, confirmed):
    """Valores truthy não substituem a confirmação explícita do cliente."""
    _state, adapter, api, _now = gateway
    with pytest.raises(GatewayError, match='Confirme'):
        api.execute(_body(api, confirmed=confirmed))
    assert adapter.calls == []


def test_expired_nonce_is_rejected_even_with_fresh_telemetry(gateway):
    """Uma intenção antiga não executa só porque o estado voltou a estar recente."""
    _state, adapter, api, now = gateway
    request = _body(api, 'camera.capture', {})
    now[0] += 10.01
    with pytest.raises(GatewayError, match='expirada'):
        api.execute(request)
    assert adapter.calls == []


def test_consumed_nonce_cannot_authorize_another_request(gateway):
    """Uma consulta autoriza no máximo um efeito."""
    _state, adapter, api, _now = gateway
    request = _body(api, 'camera.capture', {})
    assert api.execute(request)['status'] == 'submitted'
    with pytest.raises(GatewayError, match='expirada'):
        api.execute(dict(request, id='request_0002'))
    assert adapter.calls == [('camera.capture', {})]


def test_timeout_result_is_idempotent_after_nonce_expires(gateway):
    """Repetição de transporte retorna o resultado incerto sem repetir o efeito."""
    _state, adapter, api, now = gateway
    adapter.timeout = True
    request = _body(api, 'camera.capture', {})
    first = api.execute(request)
    now[0] += 20
    second = api.execute(request)
    assert first == second
    assert first['status'] == 'uncertain'
    assert adapter.calls == [('camera.capture', {})]


def test_request_id_reuse_with_different_content_is_rejected(gateway):
    """Um id não pode ser reaproveitado para enviar outro comando."""
    _state, adapter, api, _now = gateway
    api.execute(_body(api, 'camera.capture', {}))
    with pytest.raises(GatewayError, match='outro conteúdo') as error:
        api.execute(_body(api, 'camera.record', {'enabled': True}))
    assert error.value.status == 409
    assert len(adapter.calls) == 1


@pytest.mark.parametrize('mission_name,on_mission', [
    ('EXECUTANDO_INSPECIONANDO', False), ('RETORNANDO', False), ('PRONTO', True),
])
def test_cv_commands_cannot_take_over_active_mission(gateway, mission_name, on_mission):
    """Estado e posse de sessão bloqueiam recursos CV controlados pela missão."""
    state, adapter, api, _now = gateway
    state.update('mission', {'state_name': mission_name, 'on_mission': on_mission})
    with pytest.raises(GatewayError, match='controle da missão'):
        api.execute(_body(api, 'cv.record', {'enabled': False}))
    assert adapter.calls == []


def test_cv_models_reject_unavailable_equipment(gateway):
    """Estar no catálogo não é suficiente quando o peso não está disponível."""
    _state, adapter, api, _now = gateway
    with pytest.raises(GatewayError, match='indisponível'):
        api.execute(_body(api, 'cv.models', {
            'object_model': 'missing.pt', 'anomaly_model': 'anomaly.pt'}))
    assert adapter.calls == []


def test_demo_disables_commands_even_when_requested():
    """Demonstração não publica efeitos pelo adaptador."""
    adapter = FakeAdapter()
    api = MobileAPI(MobileState(demo=True), adapter, {}, enable_commands=True)
    assert api.snapshot()['commands_enabled'] is False
    with pytest.raises(GatewayError, match='somente leitura') as error:
        api.execute(_body(api, 'camera.capture', {}))
    assert error.value.status == 403
    assert adapter.calls == []


def test_session_rejects_traversal_directory_and_file_symlinks(tmp_path):
    """O leitor não sai da raiz configurada por nome ou link simbólico."""
    root = tmp_path / 'sessions'
    root.mkdir()
    outside = tmp_path / 'private'
    outside.mkdir()
    (outside / 'events.jsonl').write_text('{"private": true}\n')
    (root / 'mission_link').symlink_to(outside, target_is_directory=True)
    file_link = root / 'mission_filelink'
    file_link.mkdir()
    (file_link / 'events.jsonl').symlink_to(outside / 'events.jsonl')
    api = MobileAPI(MobileState(), FakeAdapter(), {}, sessions_dir=root)
    for name in ('../private', 'mission_../../private', '/private',
                 'mission_link', 'mission_filelink'):
        with pytest.raises(GatewayError):
            api.session(name)
    assert 'mission_link' not in [item['name'] for item in api.sessions()['sessions']]


def test_session_returns_only_recent_parseable_events(tmp_path):
    """Diários grandes ficam limitados e linhas incompletas não quebram a resposta."""
    folder = tmp_path / 'mission_valid'
    folder.mkdir()
    lines = [json.dumps({'index': index}) for index in range(250)]
    (folder / 'events.jsonl').write_text('\n'.join(lines + ['incompleto']))
    api = MobileAPI(MobileState(), FakeAdapter(), {}, sessions_dir=tmp_path)
    result = api.session('mission_valid')
    assert len(result['events']) == 199
    assert result['events'][-1] == {'index': 249}
    assert api.sessions()['sessions'][0]['name'] == 'mission_valid'
