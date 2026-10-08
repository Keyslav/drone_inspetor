"""Sinalização e vídeo reais em loopback; imagens sintéticas, sem ROS ou voo."""

import asyncio
from http.client import HTTPConnection
import json
from threading import Event, Thread

import pytest

pytest.importorskip('aiortc', reason='Dependência opcional de vídeo WebRTC')
import cv2
import numpy as np
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

from drone_inspetor.mobile_gateway.api import GatewayError, MobileAPI
from drone_inspetor.mobile_gateway.server import DemoAdapter, GatewayServer
from drone_inspetor.mobile_gateway.state import MobileState
from drone_inspetor.mobile_gateway.webrtc import LatestFrameTrack, WebRTCService


def jpeg(width=1920, height=1080):
    """Verde uniforme facilita distinguir vídeo decodificado de quadro neutro."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 1] = 180
    success, encoded = cv2.imencode('.jpg', frame)
    assert success
    return encoded.tobytes()


def request(server, path, body=None, *, token='demo', origin=None):
    connection = HTTPConnection(*server.server_address, timeout=15)
    headers = {'Authorization': 'Bearer ' + token}
    if origin:
        headers['Origin'] = origin
    if body is not None:
        headers['Content-Type'] = 'application/json'
        body = json.dumps(body)
    connection.request('POST' if body is not None else 'GET', '/api/v1/video' + path,
                       body, headers)
    response = connection.getresponse()
    result = response.status, json.loads(response.read())
    connection.close()
    return result


@pytest.fixture
def video_gateway(tmp_path):
    state = MobileState(demo=True)
    api = MobileAPI(state, DemoAdapter(), {})
    video = WebRTCService(state)
    server = GatewayServer(('127.0.0.1', 0), api, 'demo', tmp_path, video=video)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    server.server_close()
    thread.join(3)
    assert not video.thread.is_alive()
    assert not video.peers


async def make_offer(client, *, stream='camera'):
    client.addTransceiver('video', direction='recvonly')
    await client.setLocalDescription(await client.createOffer())
    return {'type': 'offer', 'sdp': client.localDescription.sdp, 'stream': stream}


def test_auth_origin_and_offer_schema(video_gateway):
    assert request(video_gateway, '', token='')[0] == 401
    assert request(video_gateway, '/offer', {}, token='')[0] == 401
    assert request(video_gateway, '/close', {}, token='')[0] == 401
    assert request(video_gateway, '/offer', {}, origin='http://another-host')[0] == 403
    status, capabilities = request(video_gateway, '')
    assert status == 200 and capabilities == {
        'webrtc': True, 'reason': '', 'fps': 15, 'max_width': 1280}
    for body in ([], {}, {'type': 'answer', 'sdp': 'v=0', 'stream': 'camera'},
                 {'type': 'offer', 'sdp': 'v=0', 'stream': 'shell'}):
        assert request(video_gateway, '/offer', body)[0] == 400
    for body in ({}, {'session_id': 1}, {'session_id': 'x', 'extra': True}):
        assert request(video_gateway, '/close', body)[0] == 400
    assert request(video_gateway, '/close', {'session_id': 'already-closed'}) == (
        200, {'closed': True})
    assert not video_gateway.video.peers


def test_video_receives_resized_frame_and_stale_source_turns_neutral(video_gateway):
    async def scenario():
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        state = video_gateway.api.state
        now = [10.0]
        state.clock = lambda: now[0]
        state.put_frame('camera', jpeg())
        received = asyncio.get_running_loop().create_future()

        @client.on('track')
        def track_received(track):
            received.set_result(track)

        try:
            body = await make_offer(client)
            status, answer = await asyncio.to_thread(request, video_gateway, '/offer', body)
            assert status == 200 and answer['session_id']
            await client.setRemoteDescription(RTCSessionDescription(
                type=answer['type'], sdp=answer['sdp']))
            track = await asyncio.wait_for(received, 5)
            frame = await asyncio.wait_for(track.recv(), 5)
            assert (frame.width, frame.height) == (1280, 720)
            assert frame.to_ndarray(format='rgb24')[50, 50, 1] > 150
            assert state.snapshot()['frames']['camera'] == {'available': True, 'age_s': 0.0}
            now[0] += 4
            assert state.snapshot()['frames']['camera'] == {'available': False, 'age_s': 4.0}
            # Podem existir pacotes de poucos quadros já em trânsito no jitter buffer.
            for _ in range(10):
                frame = await asyncio.wait_for(track.recv(), 3)
                if frame.to_ndarray(format='rgb24')[50, 50].max() < 20:
                    break
            else:
                pytest.fail('Fonte parada continuou apresentando imagem antiga como vídeo vivo')
            result = await asyncio.to_thread(request, video_gateway, '/close',
                                             {'session_id': answer['session_id']})
            assert result == (200, {'closed': True})
            assert not video_gateway.video.peers
        finally:
            await client.close()
    asyncio.run(scenario())


def test_rejects_client_media_and_invalid_sdp_without_leaking_sessions(video_gateway):
    async def scenario():
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        try:
            offer = await make_offer(client)
            sending = dict(offer, sdp=offer['sdp'].replace('a=recvonly', 'a=sendrecv'))
            assert (await asyncio.to_thread(request, video_gateway, '/offer', sending))[0] == 400
            malformed = dict(offer, sdp='m=video 9 UDP/TLS/RTP/SAVPF 96\na=recvonly\n')
            assert (await asyncio.to_thread(request, video_gateway, '/offer', malformed))[0] == 400
            assert not video_gateway.video.peers
        finally:
            await client.close()
    asyncio.run(scenario())


def test_session_limit_and_unclaimed_answer_expire():
    async def scenario():
        service = WebRTCService(MobileState(), max_sessions=1, connection_timeout=0.15)
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        try:
            offer = await make_offer(client)
            await asyncio.to_thread(service.offer, offer)
            with pytest.raises(GatewayError) as error:
                await asyncio.to_thread(service.offer, offer)
            assert error.value.status == 503
            await asyncio.sleep(0.8)
            assert not service.peers
        finally:
            await client.close()
            await asyncio.to_thread(service.close)
        assert not service.thread.is_alive()
    asyncio.run(scenario())


def test_latest_track_reuses_decoding_and_does_not_queue_frames():
    async def scenario():
        now = [10.0]
        state = MobileState(clock=lambda: now[0])
        state.put_frame('cv', jpeg(320, 240))
        track = LatestFrameTrack(state, 'cv')
        try:
            first = await track.recv()
            cached = track.cached_frame
            await track.recv()
            assert track.cached_frame is cached
            state.put_frame('cv', b'invalid')
            now[0] += 1
            state.put_frame('cv', jpeg(640, 360))
            latest = await track.recv()
            assert (first.width, first.height) == (320, 240)
            assert (latest.width, latest.height) == (640, 360)
            assert len(state.frames) == 1
        finally:
            track.stop()
    asyncio.run(scenario())


def test_shutdown_cancels_negotiation_before_stopping_loop(monkeypatch):
    entered = Event()

    async def blocked_description(_peer, _description):
        entered.set()
        await asyncio.sleep(20)

    monkeypatch.setattr(RTCPeerConnection, 'setRemoteDescription', blocked_description)

    async def scenario():
        service = WebRTCService(MobileState())
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        try:
            offer = await make_offer(client)
            pending = asyncio.create_task(asyncio.to_thread(service.offer, offer))
            assert await asyncio.to_thread(entered.wait, 3)
            await asyncio.to_thread(service.close)
            with pytest.raises(GatewayError) as error:
                await pending
            assert error.value.status == 503
            assert not service.peers and not service.thread.is_alive()
        finally:
            await client.close()
            await asyncio.to_thread(service.close)
    asyncio.run(scenario())


def test_incompatible_openssl_fails_before_spawning_thread(monkeypatch):
    from drone_inspetor.mobile_gateway import webrtc

    def broken_context(*_args):
        raise AttributeError('incompatible system cryptography')

    monkeypatch.setattr(webrtc.SSL, 'Context', broken_context)
    with pytest.raises(RuntimeError, match='run-gateway.sh'):
        WebRTCService(MobileState())


def test_browser_complete_offer_without_end_marker_does_not_leave_ice_tasks():
    """Chromium omite o marcador; trocar a câmera durante ICE não pode vazar tarefas."""
    async def scenario():
        service = WebRTCService(MobileState())
        errors = []
        service.loop.call_soon_threadsafe(
            service.loop.set_exception_handler, lambda _loop, context: errors.append(context))
        client = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        try:
            offer = await make_offer(client)
            offer['sdp'] = offer['sdp'].replace('a=end-of-candidates\r\n', '')
            await client.close()  # Operador já saiu da câmera antes de receber a resposta.
            for _ in range(4):
                answer = await asyncio.to_thread(service.offer, offer)
                await asyncio.to_thread(service.close_session, {'session_id': answer['session_id']})
            await asyncio.sleep(0.6)

            async def pending():
                return [task.get_coro().__qualname__ for task in asyncio.all_tasks()
                        if task is not asyncio.current_task() and not task.done()]

            tasks = await asyncio.wrap_future(asyncio.run_coroutine_threadsafe(pending(), service.loop))
            assert not errors, errors
            assert not tasks, tasks
        finally:
            await client.close()
            await asyncio.to_thread(service.close)
    asyncio.run(scenario())
