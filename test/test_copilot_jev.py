"""Contrato Jev sem rede: opções delimitadas, respostas inválidas e falhas."""

from copy import deepcopy
import math

from drone_inspetor.copilot.jev import ENDPOINT, JevProvider
from drone_inspetor.mobile_gateway.api import GatewayError

import pytest


@pytest.fixture
def context():
    """Catálogo do servidor e telemetria compacta para classificação."""
    return {
        'choices': [
            {'id': 'none', 'description': 'Nenhuma ação.'},
            {'id': 'start:Flare', 'description': 'Iniciar missão cadastrada Flare.'},
            {'id': 'mission.cancel', 'description': 'Cancelar a missão atual.'},
            {'id': 'camera.capture', 'description': 'Capturar imagem.'},
        ],
        'telemetry': {'mission': {'state_name': 'PRONTO'}},
    }


def response(choice='start:Flare', confidence=0.9):
    """Resposta compatível com o exemplo Choice da documentação oficial."""
    return {
        'model': 'jev-1.13.0',
        'answers': {'action': {
            'type': 'choice', 'choice': choice, 'confidence': confidence,
            'probabilities': {
                'none': 0.025, 'start:Flare': 0.925,
                'mission.cancel': 0.025, 'camera.capture': 0.025,
            },
        }},
        'usage': {'input_tokens': 300, 'output_tokens': 20},
    }


def test_jev_posts_official_choice_schema_without_executing(context):
    """IDs e descrições seguem como critérios, com endpoint e versão fixos."""
    calls = []
    original = deepcopy(context)

    def post(url, api_key, body, *, timeout):
        calls.append((url, api_key, body, timeout))
        return response()

    result = JevProvider('test-key', post=post).propose('Inspecione a Flare', context)
    assert result['choice'] == 'start:Flare'
    assert result['confidence'] == 0.9
    assert 0 < len(result['explanation']) <= 1000
    assert len(calls) == 1
    url, api_key, body, timeout = calls[0]
    assert (url, api_key, timeout) == (ENDPOINT, 'test-key', 8.0)
    assert body['model'] == 'jev-1.13.0'
    assert body['state'] == {'request': 'Inspecione a Flare', 'telemetry': context['telemetry']}
    question = body['questions']['action']
    assert question['type'] == 'choice'
    assert set(question['criteria']) == {item['id'] for item in context['choices']}
    assert question['criteria']['start:Flare'] == context['choices'][1]['description']
    assert context == original


def test_missing_none_is_added_to_remote_options(context):
    """O modelo sempre dispõe da alternativa de não propor efeito."""
    context['choices'] = context['choices'][1:]

    def post(_url, _api_key, body, **_kwargs):
        assert 'none' in body['questions']['action']['criteria']
        return response('none')

    result = JevProvider('key', post=post).propose('Como está o drone?', context)
    assert result['choice'] == 'none'


def test_no_available_action_avoids_network():
    """Um catálogo vazio não precisa de inferência remota para escolher none."""
    def forbidden_post(*_args, **_kwargs):
        pytest.fail('Não deve consultar API sem ações disponíveis.')

    result = JevProvider('key', post=forbidden_post).propose(
        'Fotografe', {'choices': [], 'telemetry': {}})
    assert result['choice'] == 'none'
    assert result['confidence'] is None


@pytest.mark.parametrize('bad_response', [
    None, [], {}, {'answers': []}, {'answers': {'action': None}},
    {'answers': {'action': {'type': 'noul', 'noul': 1}}},
    {'answers': {'action': {'type': 'choice'}}},
    {'answers': {'action': {'type': 'choice', 'choice': ['camera.capture']}}},
    {'answers': {'action': {'type': 'choice', 'choice': 'start:Inventada'}}},
    {'answers': {'action': {'type': 'choice', 'choice': 'arm'}}},
])
def test_invalid_or_out_of_catalog_response_fails_closed(context, bad_response):
    """Estruturas e comandos desconhecidos nunca se tornam uma proposta válida."""
    provider = JevProvider('key', post=lambda *_args, **_kwargs: bad_response)
    with pytest.raises(GatewayError) as error:
        provider.propose('Inicie', context)
    assert error.value.status == 502


@pytest.mark.parametrize('confidence', [True, '0.9', -0.1, 1.1, math.nan, math.inf])
def test_invalid_confidence_is_rejected(context, confidence):
    """Metadados precisam ser números finitos no intervalo documentado."""
    provider = JevProvider('key', post=lambda *_args, **_kwargs: response(confidence=confidence))
    with pytest.raises(GatewayError, match='confiança'):
        provider.propose('Inspecione a Flare', context)


@pytest.mark.parametrize('confidence', [0, 0.1, 1, None])
def test_confidence_is_informative_and_does_not_authorize(context, confidence):
    """Nenhum limiar no provedor substitui a validação independente do supervisor."""
    provider = JevProvider('key', post=lambda *_args, **_kwargs: response(confidence=confidence))
    result = provider.propose('Inspecione a Flare', context)
    assert result['choice'] == 'start:Flare'
    assert result['confidence'] == confidence


def test_transport_failure_propagates_without_retry(context):
    """Falha da consulta não vira outra ação nem uma chamada duplicada."""
    calls = []

    def failing_post(*_args, **_kwargs):
        calls.append(1)
        raise GatewayError('API indisponível.', 503)

    with pytest.raises(GatewayError, match='indisponível'):
        JevProvider('key', post=failing_post).propose('Inspecione a Flare', context)
    assert calls == [1]


def test_duplicate_options_are_rejected_before_network(context):
    """Um ID repetido não pode sobrescrever a descrição de uma ação."""
    context['choices'].append({'id': 'camera.capture', 'description': 'Outra coisa.'})
    with pytest.raises(GatewayError, match='repetida'):
        JevProvider('key', post=lambda *_args, **_kwargs: pytest.fail('Sem rede')).propose(
            'Fotografe', context)


def test_missing_key_is_an_explicit_configuration_error():
    """Não tenta consultar o serviço quando a credencial está ausente."""
    with pytest.raises(GatewayError, match='TYPESAFE_API_KEY') as error:
        JevProvider('')
    assert error.value.status == 503
