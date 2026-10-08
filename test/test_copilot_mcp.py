"""Ponte MCP limita o contrato a leitura e propostas; sem rede ou credenciais reais."""

import pytest

from drone_inspetor.copilot.mcp_server import BridgeError, GatewayClient, validate_gateway


@pytest.mark.parametrize('url', ['http://127.0.0.1:8765', 'http://192.168.1.12:8765/',
                               'https://station.example'])
def test_valid_gateway(url):
    assert validate_gateway(url) == url.rstrip('/')


@pytest.mark.parametrize('url', ['http://u:p@localhost', 'file:///etc/passwd',
                               'http://8.8.8.8', 'http://localhost/?token=abc',
                               'http://localhost/x', 'http://localhost:0'])
def test_invalid_gateway(url):
    with pytest.raises(BridgeError):
        validate_gateway(url)


def test_tools_only_send_catalog_drafts_and_strip_command_nonce():
    client = GatewayClient('http://127.0.0.1:8765', 'private-test-key')
    calls = []

    def request(path, body=None):
        calls.append((path, body))
        return {'choices': [{'id': 'start:Flare'}]}

    client._request = request
    client.request_drone_action('start:Flare', 'Inspecionar Flare')
    assert calls == [('/api/v1/copilot', None), ('/api/v1/copilot/draft', {
        'choice': 'start:Flare', 'explanation': 'Inspecionar Flare'})]
    with pytest.raises(BridgeError):
        client.request_drone_action('px4.arm', 'Não permitido')
    assert client._public({'command_nonce': 'abc', 'nested': {'token': 'secret'},
                           'value': 'private-test-key'}) == {
        'nested': {}, 'value': '[redacted]'}
