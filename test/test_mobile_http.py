"""Contrato HTTP real em loopback, sem ROS, simulador ou comandos de voo."""

from http.client import HTTPConnection
import json
from pathlib import Path
from threading import Thread

import pytest

from drone_inspetor.mobile_gateway.api import MobileAPI
from drone_inspetor.mobile_gateway.server import DemoAdapter, GatewayServer, main
from drone_inspetor.mobile_gateway.state import MobileState


@pytest.fixture
def gateway(tmp_path):
    web = tmp_path / 'web'
    web.mkdir()
    (web / 'index.html').write_text('<title>Drone Inspetor</title>')
    (tmp_path / 'private.txt').write_text('segredo')
    state = MobileState(demo=True)
    api = MobileAPI(state, DemoAdapter(), {'Flare': {}}, enable_commands=True)
    server = GatewayServer(('127.0.0.1', 0), api, 'demo', web)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    server.server_close()
    thread.join(3)


def request(server, path, *, method='GET', body=None, headers=None, token='demo'):
    connection = HTTPConnection(*server.server_address, timeout=3)
    header = {'Authorization': 'Bearer ' + token}
    if headers:
        header.update(headers)
    connection.request(method, path, body, header)
    response = connection.getresponse()
    result = response.status, response.read(), dict(response.getheaders())
    connection.close()
    return result


def test_static_available_but_api_requires_token(gateway):
    assert request(gateway, '/', token='')[0] == 200
    assert request(gateway, '/api/v1/state', token='')[0] == 401
    status, body, headers = request(gateway, '/api/v1/state')
    state = json.loads(body)
    assert status == 200
    assert state['mode'] == 'demo' and not state['commands_enabled']
    assert state['topics']['status']['health'] == 'live'
    assert headers['Cache-Control'] == 'no-store'
    assert 'Access-Control-Allow-Origin' not in headers


@pytest.mark.parametrize('path', ['/../private.txt', '/%2e%2e/private.txt',
                                  '/api/v1/sessions/../../private.txt'])
def test_paths_cannot_escape_resource_and_session_roots(gateway, path):
    status, body, _ = request(gateway, path)
    assert status == 404
    assert b'segredo' not in body


def test_demo_refuses_commands_even_with_enable_flag(gateway):
    nonce = json.loads(request(gateway, '/api/v1/state')[1])['command_nonce']
    body = json.dumps({'id': 'request_001', 'command': 'camera.capture', 'args': {}, 'nonce': nonce})
    assert request(gateway, '/api/v1/commands', method='POST', body=body,
                   headers={'Content-Type': 'application/json'})[0] == 403


@pytest.mark.parametrize('body', ['{', '{"enabled":NaN}', '[Infinity]'])
def test_invalid_json_is_rejected(gateway, body):
    assert request(gateway, '/api/v1/commands', method='POST', body=body,
                   headers={'Content-Type': 'application/json'})[0] == 400


def test_cross_origin_commands_and_unbounded_body_are_rejected(gateway):
    assert request(gateway, '/api/v1/commands', method='POST', body='{}', headers={
        'Content-Type': 'application/json', 'Origin': 'http://another-host'})[0] == 403
    assert request(gateway, '/api/v1/commands', method='POST', body='x' * 16385,
                   headers={'Content-Type': 'application/json'})[0] == 413


def test_missing_frame_is_not_reported_as_success(gateway):
    assert request(gateway, '/api/v1/frame/camera')[0] == 404
    assert request(gateway, '/api/v1/models')[0] == 200


def test_video_optional_and_jpeg_kept_without_webrtc(gateway):
    assert request(gateway, '/api/v1/video', token='')[0] == 401
    status, raw, _ = request(gateway, '/api/v1/video')
    assert status == 200 and json.loads(raw)['webrtc'] is False
    assert request(gateway, '/api/v1/video/offer', method='POST', body='{}',
                   headers={'Content-Type': 'application/json'})[0] == 503
    gateway.api.state.put_frame('camera', b'jpeg-placeholder')
    assert request(gateway, '/api/v1/frame/camera')[1] == b'jpeg-placeholder'
    frames = json.loads(request(gateway, '/api/v1/state')[1])['frames']
    assert frames['camera']['available'] is True
    assert frames['cv'] == {'available': False, 'age_s': None}


def test_token_creation_private_and_never_overwrites(tmp_path):
    token = tmp_path / 'secret'
    assert main(['--create-token', str(token)]) == 0
    previous = token.read_text()
    assert len(previous.strip()) >= 32
    assert token.stat().st_mode & 0o777 == 0o600
    with pytest.raises(SystemExit):
        main(['--create-token', str(token)])
    assert token.read_text() == previous


def test_gateway_resource_packaged():
    from unittest.mock import patch
    import runpy
    root = Path(__file__).resolve().parents[1]
    config = {}
    with patch('setuptools.setup', side_effect=lambda **kwargs: config.update(kwargs)):
        runpy.run_path(str(root / 'setup.py'))
    files = [file for _, values in config['data_files'] for file in values]
    assert 'drone_inspetor/mobile_web/index.html' in files
    assert 'drone_inspetor/mobile_web/app.js' in files
    assert 'drone_inspetor/mobile_web/copilot.js' in files


def test_copilot_disabled_and_authenticated(gateway):
    assert request(gateway, '/api/v1/copilot', token='')[0] == 401
    assert json.loads(request(gateway, '/api/v1/copilot')[1])['enabled'] is False
    assert request(gateway, '/api/v1/copilot/draft', method='POST', body='{}',
                   headers={'Content-Type': 'application/json'})[0] == 503


def test_copilot_http_draft_review_and_single_execution(gateway, tmp_path):
    from drone_inspetor.copilot.service import CopilotService

    class Recorder:
        def __init__(self):
            self.calls = []

        def execute(self, command, args):
            self.calls.append((command, args))
            return {'status': 'completed', 'message': 'Foto sintética'}

    # Apenas double local: este teste nunca conecta ao ROS.
    adapter = Recorder()
    api = MobileAPI(MobileState(), adapter, {'Flare': {}}, enable_commands=True)
    api.copilot = CopilotService(api, {}, mode='review', audit_dir=tmp_path)
    gateway.api = api

    def post(path, body, **kwargs):
        return request(gateway, '/api/v1/copilot/' + path, method='POST',
                       body=json.dumps(body), headers={'Content-Type': 'application/json'}, **kwargs)

    assert post('draft', {}, token='')[0] == 401
    assert post('draft', {'choice': 'px4.arm', 'explanation': 'Inválido'})[0] == 422
    status, raw, _ = post('draft', {'choice': 'camera.capture', 'explanation': 'Fotografar'})
    assert status == 200
    proposal = json.loads(raw)
    assert not adapter.calls
    body = {'proposal_id': proposal['id'], 'confirmed': True,
            'nonce': json.loads(request(gateway, '/api/v1/state')[1])['command_nonce']}
    assert post('confirm', dict(body, args={'mission': 'Flare'}))[0] == 400
    assert post('confirm', body)[0] == 200
    assert post('confirm', body)[0] == 200
    assert adapter.calls == [('camera.capture', {})]
