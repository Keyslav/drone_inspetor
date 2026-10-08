"""Gateway HTTP local do dashboard Android; execução ROS é opt-in e isolada da UI."""

import argparse
import hmac
import json
import mimetypes
import os
import secrets
import ssl
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import BoundedSemaphore
from urllib.parse import unquote, urlsplit

from .api import GatewayError, MobileAPI
from .state import MobileState, json_value


def resource_path(relative):
    """Usa recursos do checkout ou do share instalado pelo colcon."""
    source = Path(__file__).resolve().parents[1] / relative
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('drone_inspetor')) / relative


class DemoAdapter:
    """Demonstração somente leitura; não importa rclpy nem inicializa DDS."""

    def models(self):
        """Não apresenta pesos inexistentes como modelos carregados."""
        return {'models': [], 'current_object_model': '', 'current_anomaly_model': ''}

    def execute(self, command, args):
        """Rejeita qualquer caminho que alcance execução na demonstração."""
        raise GatewayError('Demonstração não executa comandos.', 403)

    def close(self):
        """Mantém o mesmo ciclo de vida do adaptador ROS, sem recursos externos."""
        pass


class GatewayServer(ThreadingHTTPServer):
    """Limita conexões simultâneas e o tempo de leitura de cada cliente."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, api, token, web_root, *, video=None):
        """Vincula API e recursos antes de começar a aceitar conexões."""
        self.api, self.token = api, token
        self.video = video
        self.web_root = Path(web_root).resolve()
        self.slots = BoundedSemaphore(16)
        super().__init__(address, GatewayHandler)

    def server_close(self):
        """Libera vídeo/ICE junto com o listener HTTP, inclusive nos testes."""
        try:
            super().server_close()
        finally:
            if self.video is not None:
                self.video.close()

    def get_request(self):
        """Impede que clientes HTTP incompletos retenham threads indefinidamente."""
        client, address = super().get_request()
        client.settimeout(5)
        return client, address

    def process_request(self, request, address):
        """Recusa conexões excedentes sem criar uma fila ilimitada."""
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, address):
        """Devolve a vaga mesmo se o cliente interromper a conexão."""
        try:
            super().process_request_thread(request, address)
        finally:
            self.slots.release()


class GatewayHandler(BaseHTTPRequestHandler):
    """Rotas fechadas: nenhum shell, upload de modelo ou caminho arbitrário."""

    server_version = 'DroneInspetor/2.0'

    def log_message(self, *_args):
        """Evita escrever URLs e cabeçalhos de autenticação no terminal."""
        # URLs/cabeçalhos podem conter segredos digitados pelo cliente.
        pass

    def _send(self, status, data, content_type='application/json; charset=utf-8'):
        if not isinstance(data, bytes):
            data = json.dumps(json_value(data), ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
                         "img-src 'self' blob: data: https://tile.openstreetmap.org https://*.tile.openstreetmap.org; "
                         "connect-src 'self'; media-src 'self' blob:; frame-ancestors 'none'; base-uri 'none'")
        self.end_headers()
        self.wfile.write(data)

    def _authenticate(self):
        supplied = self.headers.get('Authorization', '')
        expected = 'Bearer ' + self.server.token
        if not hmac.compare_digest(supplied.encode(), expected.encode()):
            raise GatewayError('Token ausente ou inválido.', 401)

    def do_GET(self):
        """Encaminha consultas sem permitir alterações de estado ROS."""
        self._dispatch(False)

    def do_POST(self):
        """Encaminha somente a rota explícita de comandos autenticados."""
        self._dispatch(True)

    def _dispatch(self, post):
        try:
            self._route(post)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        except GatewayError as exc:
            self._send(exc.status, {'error': str(exc)})
        except Exception as exc:
            self.server.api.state.event(f'Falha HTTP: {type(exc).__name__}', 'warning')
            self._send(500, {'error': 'Serviço indisponível; consulte os logs do gateway.'})

    def _route(self, post):
        path = unquote(urlsplit(self.path).path)
        if path.startswith('/api/'):
            self._authenticate()
            self._api_route(path, post)
            return
        if post:
            raise GatewayError('Rota inexistente.', 404)
        target = (self.server.web_root / ('index.html' if path == '/' else path.lstrip('/')))
        target = target.resolve()
        if not target.is_relative_to(self.server.web_root) or not target.is_file():
            raise GatewayError('Arquivo inexistente.', 404)
        # A pasta é exclusiva de recursos públicos da interface.
        content_type = mimetypes.guess_type(str(target))[0] or 'application/octet-stream'
        self._send(200, target.read_bytes(), content_type)

    def _api_route(self, path, post):
        api = self.server.api
        if post:
            if path not in ('/api/v1/commands', '/api/v1/copilot/propose',
                            '/api/v1/copilot/draft', '/api/v1/copilot/confirm',
                            '/api/v1/video/offer', '/api/v1/video/close'):
                raise GatewayError('Rota inexistente.', 404)
            origin = self.headers.get('Origin')
            if origin:
                parsed = urlsplit(origin)
                if parsed.scheme not in ('http', 'https') or parsed.netloc != self.headers.get('Host'):
                    raise GatewayError('Origem da requisição não permitida.', 403)
            if self.headers.get_content_type() != 'application/json':
                raise GatewayError('Envie application/json.', 415)
            if self.headers.get('Transfer-Encoding'):
                raise GatewayError('Transfer-Encoding não suportado.')
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                raise GatewayError('Tamanho inválido.')
            max_body = 65536 if path == '/api/v1/video/offer' else 16384
            if not 0 < length <= max_body:
                raise GatewayError('Corpo vazio ou grande demais.', 413)
            try:
                body = json.loads(self.rfile.read(length), parse_constant=self._invalid_number)
            except (ValueError, UnicodeDecodeError):
                raise GatewayError('JSON inválido.')
            if path == '/api/v1/commands':
                result = api.execute(body)
            elif path.startswith('/api/v1/video/'):
                if self.server.video is None:
                    raise GatewayError('WebRTC desativado no gateway; use --webrtc ou JPEG.', 503)
                operation = (self.server.video.offer if path.endswith('/offer')
                             else self.server.video.close_session)
                result = operation(body)
            else:
                if api.copilot is None:
                    raise GatewayError('Copiloto desativado no gateway; use --copilot.', 503)
                operation = {'/api/v1/copilot/propose': api.copilot.propose,
                             '/api/v1/copilot/draft': api.copilot.draft,
                             '/api/v1/copilot/confirm': api.copilot.confirm}[path]
                result = operation(body)
            self._send(200, result)
            return
        if path == '/api/v1/state':
            self._send(200, api.snapshot())
        elif path == '/api/v1/video':
            self._send(200, self.server.video.capabilities() if self.server.video else {
                'webrtc': False, 'reason': 'Gateway iniciado sem --webrtc; JPEG disponível.',
                'fps': 15, 'max_width': 1280})
        elif path == '/api/v1/copilot':
            self._send(200, api.copilot.status() if api.copilot else {
                'enabled': False, 'mode': 'off', 'providers': [],
                'choices': [], 'proposals': []})
        elif path == '/api/v1/missions':
            self._send(200, {'missions': api.missions})
        elif path == '/api/v1/models':
            try:
                self._send(200, api.models())
            except TimeoutError:
                raise GatewayError('Catálogo CV sem resposta.', 503)
        elif path == '/api/v1/sessions':
            self._send(200, api.sessions())
        elif path.startswith('/api/v1/sessions/'):
            self._send(200, api.session(path.removeprefix('/api/v1/sessions/')))
        elif path.startswith('/api/v1/frame/'):
            key = path.removeprefix('/api/v1/frame/')
            data = api.state.frame(key)
            if data is None:
                raise GatewayError('Imagem ausente ou desatualizada.', 404)
            self._send(200, data, 'image/jpeg')
        else:
            raise GatewayError('Rota inexistente.', 404)

    @staticmethod
    def _invalid_number(value):
        raise ValueError(value)


def main(argv=None):
    """Valida opções e inicia o gateway com fechamento dos recursos ao sair."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--demo', action='store_true', help='Dados fictícios, sem ROS e sem comandos')
    parser.add_argument('--webrtc', action='store_true',
                        help='Vídeo WebRTC opcional (requer aiortc); JPEG continua disponível')
    parser.add_argument('--enable-commands', action='store_true')
    parser.add_argument('--copilot', action='store_true', help='Habilita propostas de IA/MCP')
    parser.add_argument('--copilot-mode', choices=('shadow', 'review'), default='shadow')
    parser.add_argument('--copilot-log-dir', default='~/Drone_Inspetor_IA')
    parser.add_argument('--token-file', type=Path)
    parser.add_argument('--create-token', type=Path, metavar='ARQUIVO')
    parser.add_argument('--missions-file', type=Path)
    parser.add_argument('--sessions-dir', default='~/Drone_Inspetor_Missoes')
    parser.add_argument('--media-dir', default='~/Drone_Inspetor_Mobile')
    parser.add_argument('--cert-file', type=Path)
    parser.add_argument('--key-file', type=Path)
    args, ros_args = parser.parse_known_args(argv)
    if ros_args and ros_args[0] != '--ros-args':
        parser.error(f'Argumentos desconhecidos: {ros_args}')
    if args.create_token:
        path = args.create_token.expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(descriptor, 'w') as stream:
                stream.write(secrets.token_urlsafe(32) + '\n')
        except OSError as exc:
            parser.error(f'Não foi possível criar token: {exc}')
        print(f'Token criado em {path}; mantenha o arquivo privado.')
        return 0
    token = os.environ.get('DRONE_MOBILE_TOKEN', '')
    if args.token_file:
        try:
            token = args.token_file.expanduser().read_text().strip()
        except OSError as exc:
            parser.error(f'Não foi possível ler token: {exc}')
    if args.demo and not token:
        token = 'demo'
    if not token or (not args.demo and len(token) < 32):
        parser.error('Use --token-file com token de pelo menos 32 caracteres; gere com --create-token.')
    if bool(args.cert_file) != bool(args.key_file):
        parser.error('--cert-file e --key-file devem ser usados juntos.')
    from drone_inspetor.missions.repository import MissionRepository
    adapter = server = video = None
    try:
        missions = MissionRepository(args.missions_file or resource_path('missions/missions.json'))
        missions.load()
        state = MobileState(demo=args.demo)
        if args.webrtc:
            try:
                from .webrtc import WebRTCService
            except ImportError as exc:
                raise RuntimeError('WebRTC requer as dependências opcionais: instale '
                                   'requirements-mobile-webrtc.txt no ambiente Python do gateway.') from exc
            video = WebRTCService(state)
        if args.demo:
            adapter = DemoAdapter()
        else:
            from .ros_adapter import ROSAdapter
            adapter = ROSAdapter(state, media_dir=args.media_dir, ros_args=ros_args,
                                 video_fps=15 if args.webrtc else None)
        api = MobileAPI(state, adapter, missions.as_mapping(),
                        enable_commands=args.enable_commands, sessions_dir=args.sessions_dir)
        if args.copilot:
            from drone_inspetor.copilot.providers import configured_providers
            from drone_inspetor.copilot.service import CopilotService
            api.copilot = CopilotService(api, configured_providers(), mode=args.copilot_mode,
                                         audit_dir=args.copilot_log_dir)
        server = GatewayServer((args.host, args.port), api, token, resource_path('mobile_web'),
                               video=video)
        if args.cert_file:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(str(args.cert_file), str(args.key_file))
            server.socket = context.wrap_socket(server.socket, server_side=True)
        protocol = 'https' if args.cert_file else 'http'
        print(f'Dashboard móvel: {protocol}://{args.host}:{args.port}', flush=True)
        print('Vídeo: WebRTC + JPEG.' if video else 'Vídeo: JPEG.', flush=True)
        print('Demonstração: token demo.' if args.demo and token == 'demo' else
              'Autenticação obrigatória; token não será impresso.', flush=True)
        state.event('Gateway iniciado em ' + ('demonstração.' if args.demo else 'ROS.'))
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'Falha ao iniciar gateway: {exc}', file=sys.stderr)
        return 1
    finally:
        if server:
            server.server_close()
        elif video:
            video.close()
        if adapter:
            adapter.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
