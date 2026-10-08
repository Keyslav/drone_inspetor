"""Inferência, revisão e replay testados sem API externa nem ROS."""

import json

import pytest

from drone_inspetor.copilot.providers import OpenAIProvider, configured_providers
from drone_inspetor.copilot.service import CopilotService
from drone_inspetor.mobile_gateway.api import GatewayError, MobileAPI
from drone_inspetor.mobile_gateway.state import MobileState


class Adapter:
    def __init__(self):
        self.calls = []
        self.timeout = False

    def execute(self, command, args):
        self.calls.append((command, args))
        if self.timeout:
            raise TimeoutError()
        return {'status': 'submitted', 'message': 'Recebido pelo double'}


@pytest.fixture
def rig(tmp_path):
    now = [100.]
    state = MobileState(clock=lambda: now[0])
    for key in ('drone', 'mission', 'status'):
        state.update(key, {'state_name': 'PRONTO', 'is_armed': False, 'nav_state': 14})
    adapter = Adapter()
    api = MobileAPI(state, adapter, {'Flare': {'alt': 99}}, enable_commands=True,
                    clock=lambda: now[0])
    service = CopilotService(api, {}, mode='review', clock=lambda: now[0], audit_dir=tmp_path)
    return state, api, adapter, service, now


def confirmation(api, draft):
    return {'proposal_id': draft['id'], 'nonce': api.snapshot()['command_nonce'], 'confirmed': True}


def test_proposal_does_not_execute_and_confirmation_is_idempotent(rig):
    _state, api, adapter, service, _now = rig
    draft = service.draft({'choice': 'start:Flare', 'explanation': 'Pedido explícito do operador'})
    assert not adapter.calls
    body = confirmation(api, draft)
    assert service.confirm(body)['status'] == 'submitted'
    assert service.confirm(body)['status'] == 'submitted'
    assert adapter.calls == [('mission.start', {'mission': 'Flare'})]
    assert not service.status()['proposals'][0]['can_execute']


@pytest.mark.parametrize('condition', ['shadow', 'readonly', 'expired', 'changed', 'stale'])
def test_proposal_cannot_bypass_current_conditions(rig, condition):
    state, api, adapter, service, now = rig
    draft = service.draft({'choice': 'start:Flare', 'explanation': 'Teste'})
    if condition == 'shadow':
        service.mode = 'shadow'
    elif condition == 'readonly':
        api.enabled = False
    elif condition == 'expired':
        now[0] += 61
    elif condition == 'changed':
        state.update('status', {'nav_state': 3})
    else:
        now[0] += 4
    with pytest.raises(GatewayError):
        service.confirm(confirmation(api, draft))
    assert not adapter.calls


def test_confirmation_does_not_accept_replaced_command_or_forged_arguments(rig):
    _state, api, adapter, service, _now = rig
    draft = service.draft({'choice': 'camera.capture', 'explanation': 'Foto'})
    with pytest.raises(GatewayError):
        service.confirm(dict(confirmation(api, draft), command='mission.start'))
    with pytest.raises(GatewayError):
        service.confirm(dict(confirmation(api, draft), confirmed='true'))
    with pytest.raises(GatewayError):
        service.draft({'choice': 'px4.arm', 'explanation': 'Arme'})
    assert not adapter.calls


def test_uncertain_confirmation_never_repeats_effect(rig):
    _state, api, adapter, service, _now = rig
    adapter.timeout = True
    draft = service.draft({'choice': 'camera.capture', 'explanation': 'Foto'})
    body = confirmation(api, draft)
    assert service.confirm(body)['status'] == 'uncertain'
    assert service.confirm(body)['status'] == 'uncertain'
    assert len(adapter.calls) == 1


def test_demo_and_missing_keys_do_not_turn_into_real_inference(rig, monkeypatch):
    _state, api, adapter, service, _now = rig
    for key in ('TYPESAFE_API_KEY', 'OPENAI_API_KEY', 'DRONE_LLM_MODEL'):
        monkeypatch.delenv(key, raising=False)
    service.providers = configured_providers()
    available = {item['name']: item['configured'] for item in service.status()['providers']}
    assert available == {'demo': True, 'jev': False, 'openai': False}
    draft = service.propose({'provider': 'demo', 'prompt': 'iniciar Flare'})
    assert draft['choice'] == 'start:Flare' and not draft['can_execute']
    with pytest.raises(GatewayError):
        service.confirm(confirmation(api, draft))
    assert not adapter.calls


def test_context_is_small_and_has_no_precise_location_or_images(rig):
    state, _api, _adapter, service, _now = rig
    state.update('drone', {'state_name': 'PRONTO', 'current_latitude': -22.6})
    context = json.dumps(service.context())
    assert 'current_latitude' not in context and '-22.6' not in context
    assert 'start:Flare' in context


def test_openai_uses_enum_and_rejects_incomplete_or_refused_responses():
    captured = []
    response = {'status': 'completed', 'output': [{'type': 'message', 'content': [
        {'type': 'output_text', 'text': '{"choice":"none","explanation":"Dados insuficientes"}'}]}]}

    def post(url, key, body):
        captured.append(body)
        return response

    provider = OpenAIProvider('test-key', 'configured-model', post=post)
    context = {'choices': [{'id': 'none', 'description': 'Não executar'}], 'telemetry': {}}
    assert provider.propose('como está?', context)['choice'] == 'none'
    assert captured[0]['store'] is False
    assert captured[0]['text']['format']['schema']['properties']['choice']['enum'] == ['none']
    response['status'] = 'incomplete'
    with pytest.raises(GatewayError):
        provider.propose('teste', context)
    response['status'] = 'completed'
    response['output'][0]['content'] = [{'type': 'refusal', 'refusal': 'No'}]
    with pytest.raises(GatewayError):
        provider.propose('teste', context)
