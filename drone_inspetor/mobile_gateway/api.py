"""API móvel com comandos finitos, validade curta e resultado não presumido."""

import json
import re
import secrets
import time
from collections import OrderedDict
from pathlib import Path
from threading import RLock

from .state import json_value


COMMANDS = (
    'mission.start', 'mission.cancel', 'camera.capture', 'camera.record',
    'cv.models', 'cv.record', 'cv.anomaly',
)


class GatewayError(Exception):
    """Erro previsível traduzido para HTTP sem expor traceback ao cliente."""

    def __init__(self, message, status=400):
        """Associa a mensagem pública ao código de resposta HTTP."""
        super().__init__(message)
        self.status = status


class MobileAPI:
    """Separa validação de rede das operações ROS/arquivos do adaptador."""

    def __init__(self, state, adapter, missions, *, enable_commands=False,
                 sessions_dir=None, clock=time.monotonic):
        """Configura os limites de comando sem habilitar efeitos na demonstração."""
        self.state, self.adapter, self.missions = state, adapter, missions
        self.enabled = enable_commands and not state.demo
        self.sessions_dir = Path(sessions_dir).expanduser().resolve() if sessions_dir else None
        self.clock = clock
        self.lock = RLock()
        self.requests = OrderedDict()
        self.nonces = OrderedDict()
        self.copilot = None

    def snapshot(self):
        """Retorna o estado atual com uma autorização curta de uso único."""
        snapshot = self.state.snapshot(commands_enabled=self.enabled, capabilities=COMMANDS)
        with self.lock:
            nonce = secrets.token_urlsafe(18)
            self.nonces[nonce] = self.clock()
            while len(self.nonces) > 128:
                self.nonces.popitem(last=False)
        snapshot['command_nonce'] = nonce
        return snapshot

    def models(self):
        """Consulta o catálogo publicado pelo serviço CV."""
        return self.adapter.models()

    def execute(self, body):
        """Valida a intenção e impede a repetição de efeitos após timeout."""
        if not isinstance(body, dict):
            raise GatewayError('Esperado objeto JSON.')
        if set(body) - {'id', 'command', 'args', 'confirmed', 'nonce'}:
            raise GatewayError('Campos de comando desconhecidos.')
        request_id = body.get('id')
        if not isinstance(request_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}', request_id):
            raise GatewayError('id deve identificar uma solicitação única.')
        command, args = body.get('command'), body.get('args', {})
        if command not in COMMANDS or not isinstance(args, dict):
            raise GatewayError('Comando ou argumentos inválidos.')
        try:
            fingerprint = json.dumps(body, sort_keys=True, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise GatewayError('Comando deve conter apenas valores JSON finitos.') from exc
        with self.lock:
            previous = self.requests.get(request_id)
            if previous:
                if previous[0] != fingerprint:
                    raise GatewayError('id reutilizado com outro conteúdo.', 409)
                return previous[1]
            if not self.enabled:
                raise GatewayError('Controles remotos desabilitados; sessão somente leitura.', 403)
            nonce = body.get('nonce')
            if not isinstance(nonce, str):
                raise GatewayError('Atualize a telemetria antes de enviar.', 409)
            issued = self.nonces.pop(nonce, None)
            if issued is None or self.clock() - issued > 10:
                raise GatewayError(
                    'Solicitação expirada; confira o estado e tente manualmente.', 409)
            self._validate(command, args, body.get('confirmed') is True)
            # Reserva antes do efeito: timeout de transporte não permite replay.
            result = {'id': request_id, 'status': 'uncertain',
                      'message': 'Operação sem resultado confirmado; consulte o estado.'}
            self.requests[request_id] = (fingerprint, result)
            while len(self.requests) > 512:
                self.requests.popitem(last=False)
            try:
                response = self.adapter.execute(command, args)
                result = dict(response, id=request_id)
            except GatewayError as exc:
                result = {'id': request_id, 'status': 'rejected', 'message': str(exc)}
            except TimeoutError:
                result['message'] = (
                    'Tempo de resposta excedido; resultado incerto. Não reenviar automaticamente.')
            except Exception:
                # O efeito pode ter ocorrido antes da exceção: nunca chamar de rejeição certa.
                result['message'] = (
                    'Falha ao obter resultado; confira telemetria e logs antes de repetir.')
            self.requests[request_id] = (fingerprint, result)
            level = 'warning' if result['status'] in ('uncertain', 'rejected') else 'info'
            self.state.event(f'{command}: {result["message"]}', level)
            return result

    def _validate(self, command, args, confirmed):
        expected = {
            'mission.start': {'mission'}, 'mission.cancel': set(),
            'camera.capture': set(), 'camera.record': {'enabled'},
            'cv.models': {'object_model', 'anomaly_model'},
            'cv.record': {'enabled'}, 'cv.anomaly': {'enabled'},
        }[command]
        if set(args) != expected:
            raise GatewayError(f'Argumentos esperados: {sorted(expected)}')
        if 'enabled' in args and type(args['enabled']) is not bool:
            raise GatewayError('enabled deve ser booleano.')
        topics = self.state.snapshot()['topics']
        mission = topics['mission']
        if command.startswith('mission.'):
            if not confirmed:
                raise GatewayError('Confirme explicitamente a ação de missão.', 409)
            if any(topics[key]['health'] != 'live' for key in ('mission', 'drone', 'status')):
                raise GatewayError('Telemetria de missão/drone/PX4 ausente ou desatualizada.', 409)
            if command == 'mission.start':
                name = args['mission']
                if not isinstance(name, str) or name not in self.missions:
                    raise GatewayError('Missão não consta no catálogo do gateway.')
                if mission['values'].get('state_name') != 'PRONTO':
                    raise GatewayError('MissionNode não está PRONTO.', 409)
            elif not (str(mission['values'].get('state_name', '')).startswith('EXECUTANDO') or
                      mission['values'].get('state_name') == 'INSPECAO_FINALIZADA'):
                raise GatewayError('A missão não está em uma etapa cancelável.', 409)
        if command.startswith('cv.'):
            if mission['health'] != 'live':
                raise GatewayError('Estado da missão indisponível para alterar recursos CV.', 409)
            active_state = str(mission['values'].get('state_name', '')).startswith(
                ('EXECUTANDO', 'RETORNANDO'))
            if mission['values'].get('on_mission') or active_state:
                raise GatewayError('Recurso CV sob controle da missão em andamento.', 409)
        if command == 'cv.models':
            entries = self.models()['models']
            for field, kind in (('object_model', 'equipment'), ('anomaly_model', 'anomaly')):
                name = args[field]
                if not isinstance(name, str) or not any(
                        item.get('file_name') == name and item.get('object_type') == kind
                        and item.get('available') is True for item in entries):
                    raise GatewayError(f'Modelo indisponível para {kind}.')

    def sessions(self):
        """Lista até cinquenta diretórios de missão, ignorando links simbólicos."""
        if not self.sessions_dir or not self.sessions_dir.is_dir():
            return {'sessions': []}
        entries = []
        for path in self.sessions_dir.glob('mission_*'):
            if path.is_symlink() or not path.is_dir():
                continue
            entries.append({'name': path.name, 'modified_at': path.stat().st_mtime})
        recent = sorted(entries, key=lambda item: item['modified_at'], reverse=True)
        return {'sessions': recent[:50]}

    def session(self, name):
        """Lê a cauda limitada de um diário contido na raiz configurada."""
        if (not self.sessions_dir or not isinstance(name, str) or
                not re.fullmatch(r'mission_[A-Za-z0-9_.-]+', name)):
            raise GatewayError('Sessão inválida.', 404)
        folder = self.sessions_dir / name
        target = folder / 'events.jsonl'
        if folder.is_symlink() or target.is_symlink() or not target.is_file():
            raise GatewayError('Diário não encontrado.', 404)
        if not target.resolve().is_relative_to(self.sessions_dir):
            raise GatewayError('Diário fora do diretório de sessões.', 403)
        with target.open('rb') as stream:
            size = stream.seek(0, 2)
            stream.seek(max(0, size - 256 * 1024))
            if size > 256 * 1024:
                stream.readline()
            lines = stream.read().decode('utf-8', 'replace').splitlines()[-200:]
        events = []
        for line in lines:
            try:
                events.append(json_value(json.loads(line)))
            except (json.JSONDecodeError, ValueError):
                continue
        return {'events': events}
