"""Vídeo WebRTC opcional: envia o último quadro ROS, sem receber comandos ou mídia.

A sinalização usa o HTTP autenticado existente. ICE usa somente endereços locais;
redes que bloqueiam UDP podem continuar usando JPEG. O loop dedicado evita que a
negociação ou a codificação bloqueiem callbacks ROS e requisições de telemetria.
"""

import asyncio
from concurrent.futures import CancelledError, TimeoutError as FutureTimeout
from fractions import Fraction
import io
import secrets
from threading import Event, RLock, Thread
import time

import av
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aioice.ice import CandidatePair, ICE_FAILED
from OpenSSL import SSL

from .api import GatewayError


FPS = 15
MAX_WIDTH = 1280
STREAMS = ('camera', 'cv', 'depth')


async def _finish_ice_checks(peer):
    """Drena verificações STUN antes de fechar UDP (aiortc 1.15/aioice 0.10.2).

    aioice.close() fecha sockets antes de cancelar verificações em andamento.
    Na troca rápida de câmera isso deixa retransmissões apontando a sockets já
    fechados. Esta compatibilidade fica restrita a este helper e às versões
    fixadas em requirements-mobile-webrtc.txt; o teste de regressão verifica
    que nenhuma tarefa ou exceção permanece após abandonar a negociação.
    """
    pending = set()
    for transceiver in peer.getTransceivers():
        transport = transceiver.receiver.transport.transport
        await transport.addRemoteCandidate(None)
        connection = transport._connection
        for pair in connection._check_list:
            # Impede check_periodic de iniciar outro par enquanto cancelamos.
            pair.state = CandidatePair.State.FAILED
            if pair.task is not None and not pair.task.done():
                pair.task.cancel()
                pending.add(pair.task)
        if connection._check_list and not connection._check_list_done:
            connection._check_list_done = True
            connection._check_list_state.put_nowait(ICE_FAILED)
    if pending:
        # Cancelar Transaction.run executa seu finally e remove os timers STUN.
        await asyncio.gather(*pending, return_exceptions=True)


class LatestFrameTrack(VideoStreamTrack):
    """Sem fila: descarta amostras antigas e limita a codificação a 15 quadros/s."""

    def __init__(self, state, stream):
        super().__init__()
        self.state, self.stream = state, stream
        self.started = time.monotonic()
        self.next_frame = self.started
        self.cached_at = None
        self.cached_frame = None
        self.blank = av.VideoFrame(640, 360, 'yuv420p')
        for index, plane in enumerate(self.blank.planes):
            plane.update(bytes([16 if index == 0 else 128]) * plane.buffer_size)

    @staticmethod
    def _decode(data):
        with av.open(io.BytesIO(data), format='mjpeg') as container:
            decoded = next(container.decode(video=0))
            scale = min(1.0, MAX_WIDTH / decoded.width, 1280 / decoded.height)
            width = max(2, int(decoded.width * scale) // 2 * 2)
            height = max(2, int(decoded.height * scale) // 2 * 2)
            return decoded.reformat(width=width, height=height, format='yuv420p')

    async def recv(self):
        await asyncio.sleep(max(0, self.next_frame - time.monotonic()))
        now = time.monotonic()
        self.next_frame = now + 1 / FPS
        sample = self.state.frame_sample(self.stream)
        if sample is None:
            # Nunca apresenta a última imagem de uma câmera parada como vídeo vivo.
            frame = self.blank
        else:
            data, at = sample
            if at != self.cached_at:
                self.cached_at = at
                try:
                    self.cached_frame = await asyncio.to_thread(self._decode, data)
                except Exception:
                    self.cached_frame = None
                    self.state.event('Imagem inválida no vídeo WebRTC: ' + self.stream, 'warning')
            frame = self.cached_frame or self.blank
        frame.pts = int((time.monotonic() - self.started) * 90000)
        frame.time_base = Fraction(1, 90000)
        return frame


class WebRTCService:
    """No máximo quatro espectadores; cada conexão possui somente um vídeo de saída."""

    def __init__(self, state, *, max_sessions=4, connection_timeout=20):
        try:
            # PYTHONPATH do ROS pode colocar cryptography do apt antes da versão
            # da venv. Falhar aqui evita anunciar vídeo e quebrar só após o SDP.
            SSL.Context(SSL.DTLS_METHOD)
        except Exception as exc:
            raise RuntimeError('Dependências WebRTC incompatíveis. Use mobile/run-gateway.sh '
                               'para priorizar a venv antes dos pacotes Python do ROS.') from exc
        self.state = state
        self.max_sessions = max_sessions
        self.connection_timeout = connection_timeout
        self.peers = {}
        self.loop = asyncio.new_event_loop()
        self.ready = Event()
        self.lifecycle_lock = RLock()
        self.closed = False
        self.thread = Thread(target=self._run_loop, name='mobile-webrtc', daemon=True)
        self.thread.start()
        if not self.ready.wait(3):
            raise RuntimeError('Não foi possível iniciar o vídeo WebRTC.')

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.ready.set()
        try:
            self.loop.run_forever()
        finally:
            self.loop.run_until_complete(self.loop.shutdown_asyncgens())
            self.loop.run_until_complete(self.loop.shutdown_default_executor())
            self.loop.close()

    @staticmethod
    def capabilities():
        return {'webrtc': True, 'reason': '', 'fps': FPS, 'max_width': MAX_WIDTH}

    def _submit(self, coroutine, *, timeout=12):
        with self.lifecycle_lock:
            if self.closed:
                coroutine.close()
                raise GatewayError('Vídeo WebRTC encerrado.', 503)
            future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)
        try:
            return future.result(timeout)
        except FutureTimeout:
            future.cancel()
            raise GatewayError('Negociação WebRTC excedeu o prazo; tente JPEG.', 504)
        except CancelledError:
            raise GatewayError('Negociação WebRTC encerrada.', 503)

    def offer(self, body):
        """Aceita uma oferta de recepção; não habilita áudio, dados ou envio do cliente."""
        if (not isinstance(body, dict) or set(body) != {'type', 'sdp', 'stream'}
                or body.get('type') != 'offer' or body.get('stream') not in STREAMS
                or not isinstance(body.get('sdp'), str) or not 1 <= len(body['sdp']) <= 60000):
            raise GatewayError('Oferta WebRTC inválida.')
        # A UI oferece uma única transceiver recvonly. Recusar demais mídias reduz
        # tanto recursos quanto ambiguidades sobre o que o gateway pode receber.
        lines = body['sdp'].replace('\r\n', '\n').splitlines()
        media = [line for line in lines if line.startswith('m=')]
        directions = [line for line in lines if line in
                      ('a=recvonly', 'a=sendonly', 'a=sendrecv', 'a=inactive')]
        if (len(media) != 1 or not media[0].startswith('m=video ')
                or directions != ['a=recvonly']):
            raise GatewayError('WebRTC aceita somente um vídeo recvonly, sem áudio ou dados.')
        return self._submit(self._offer(body))

    async def _offer(self, body):
        if self.closed:
            raise GatewayError('Vídeo WebRTC encerrado.', 503)
        if len(self.peers) >= self.max_sessions:
            raise GatewayError('Limite de espectadores WebRTC atingido; use JPEG.', 503)
        session_id = secrets.token_urlsafe(24)
        peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        track = LatestFrameTrack(self.state, body['stream'])
        entry = {'peer': peer, 'track': track, 'watchdog': None,
                 'negotiation': asyncio.current_task()}
        self.peers[session_id] = entry

        @peer.on('connectionstatechange')
        async def connection_changed():
            if peer.connectionState in ('failed', 'closed'):
                await self._close_peer(session_id)

        try:
            await peer.setRemoteDescription(RTCSessionDescription(sdp=body['sdp'], type='offer'))
            # Esta API recebe a oferta completa (sem trickle ICE). Chromium pode
            # omitir a=end-of-candidates mesmo após concluir a coleta. Sem o fim
            # explícito, aioice continua aguardando candidatos ao fechar a aba.
            await peer.addIceCandidate(None)
            peer.addTrack(track)
            await peer.setLocalDescription(await peer.createAnswer())
            entry['watchdog'] = asyncio.create_task(self._watch_connection(session_id))
            return {'type': peer.localDescription.type, 'sdp': peer.localDescription.sdp,
                    'session_id': session_id}
        except BaseException as exc:
            await self._close_peer(session_id)
            if isinstance(exc, asyncio.CancelledError):
                raise
            raise GatewayError('Não foi possível negociar o vídeo WebRTC.', 400) from exc
        finally:
            entry['negotiation'] = None

    async def _watch_connection(self, session_id):
        started = time.monotonic()
        connected_once = False
        disconnected_at = None
        while session_id in self.peers:
            peer = self.peers[session_id]['peer']
            state = peer.connectionState
            now = time.monotonic()
            connected_once = connected_once or state == 'connected'
            if state in ('failed', 'closed'):
                break
            if not connected_once and now - started > self.connection_timeout:
                break
            if state == 'disconnected':
                disconnected_at = disconnected_at or now
                if now - disconnected_at > 10:
                    break
            else:
                disconnected_at = None
            # ICE possui consentimento periódico: desaparecimento do navegador
            # resulta em failed, mesmo se o POST de encerramento não chegar.
            await asyncio.sleep(0.5)
        await self._close_peer(session_id)

    def close_session(self, body):
        if (not isinstance(body, dict) or set(body) != {'session_id'}
                or not isinstance(body.get('session_id'), str)
                or not 1 <= len(body['session_id']) <= 100):
            raise GatewayError('Identificador de sessão WebRTC inválido.')
        self._submit(self._close_peer(body['session_id']))
        return {'closed': True}

    async def _close_peer(self, session_id):
        entry = self.peers.pop(session_id, None)
        if entry is None:
            return
        watchdog = entry['watchdog']
        if watchdog is not None and watchdog is not asyncio.current_task():
            watchdog.cancel()
        negotiation = entry['negotiation']
        if negotiation is not None and negotiation is not asyncio.current_task():
            negotiation.cancel()
            await asyncio.gather(negotiation, return_exceptions=True)
        entry['track'].stop()
        await _finish_ice_checks(entry['peer'])
        await entry['peer'].close()

    async def _close_all(self):
        await asyncio.gather(*(self._close_peer(key) for key in list(self.peers)))

    def close(self):
        """Fecha sockets/encoders antes de terminar o loop e seu executor auxiliar."""
        with self.lifecycle_lock:
            if self.closed:
                return
            self.closed = True
            closing = asyncio.run_coroutine_threadsafe(self._close_all(), self.loop)
        try:
            closing.result(12)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(5)
