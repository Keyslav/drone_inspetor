"""Inicia os launchers ROS existentes e observa a infraestrutura externa.

Gazebo, PX4, MicroXRCEAgent e QGroundControl são administrados fora daqui.
Somente processos iniciados por este controlador podem ser encerrados por ele.
"""

import fcntl
import json
import os
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ModeSpec:
    """Descrição de um modo oferecido pelas duas interfaces."""

    label: str
    description: str
    launch_file: str


MODE_SPECS = {
    'sim': ModeSpec(
        'Projeto na simulação',
        'Aplicação, dashboard e bridges para Gazebo/PX4 já iniciados.',
        'dashboard_launch.py'),
    'companion': ModeSpec(
        'Companion',
        'Controle e processamento sem janela, com relógio real.',
        'drone_inspetor_launch.py'),
    'dashboard': ModeSpec('Somente dashboard',
                          'Janela conectada aos nós já existentes.',
                          'dashboard_launch.py'),
}

_APPLICATION_NODES = (
    'camera', 'cv', 'depth', 'lidar', 'drone', 'mission', 'dashboard')
_CLOCK_CACHE = {'at': 0.0, 'result': None}
_CLOCK_LOCK = threading.Lock()


def _processes():
    """Lê nomes e argumentos sem executar `ps` ou iniciar o daemon ROS."""
    found = []
    try:
        entries = os.scandir('/proc')
    except OSError:
        return found
    with entries:
        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                raw = (Path(entry.path) / 'cmdline').read_bytes()
                args = [part.decode('utf-8', 'replace')
                        for part in raw.split(b'\0') if part]
                if args:
                    found.append((int(entry.name), args))
            except (OSError, ValueError):
                continue
    return found


def _executable(args):
    return Path(args[0]).name.lower() if args else ''


def _bridge_processes(processes):
    names = {_executable(args) for _, args in processes}
    return bool({'parameter_bridge', 'image_bridge'} & names)


def _launch_process(args):
    """Retorna launcher e argumentos de um comando ros2 launch."""
    for index in range(len(args) - 3):
        if (Path(args[index]).name == 'ros2' and args[index + 1:index + 3]
                == ['launch', 'drone_inspetor']):
            return args[index + 3], args[index + 4:]
    return None, []


def node_defaults(mode):
    """Devolve uma seleção nova, sem compartilhar opções entre perfis."""
    if mode not in MODE_SPECS:
        raise ValueError(f'Modo desconhecido: {mode!r}')
    return {node: (mode == 'sim' or
                   (mode == 'companion' and node != 'dashboard') or
                   (mode == 'dashboard' and node == 'dashboard'))
            for node in _APPLICATION_NODES}


def _selected_nodes(mode, overrides=None):
    selected = node_defaults(mode)
    for key, value in (overrides or {}).items():
        if key not in {f'with_{node}' for node in _APPLICATION_NODES}:
            raise ValueError(f'Opção de nó desconhecida: {key}')
        if value is not None and not isinstance(value, bool):
            raise ValueError(f'{key} deve ser None, True ou False')
        if value is not None:
            selected[key[5:]] = value
    return selected


def _external_launch_conflict(mode, processes, managed_pid=None, *, nodes=None):
    selected = node_defaults(mode) if nodes is None else nodes
    requested = {node for node, enabled in selected.items() if enabled}
    for pid, args in processes:
        if pid == managed_pid:
            continue
        for arg in args[:2]:
            executable = Path(arg).name
            if (executable.endswith('_node') and
                    'drone_inspetor' in Path(arg).parts
                    and executable[:-5] in requested):
                return pid
        for index in range(len(args) - 3):
            if (Path(args[index]).name == 'ros2' and
                    args[index + 1:index + 3] == ['run', 'drone_inspetor'] and
                    args[index + 3].endswith('_node') and
                    args[index + 3][:-5] in requested):
                return pid
        launch_file, flags = _launch_process(args)
        if launch_file not in (
                'dashboard_launch.py', 'drone_inspetor_launch.py'):
            continue
        # Os dois launchers iniciam todos os nós por padrão. ROS aceita
        # booleanos sem distinção de maiúsculas; prevalece a última opção.
        options = dict(flag.split(':=', 1) for flag in flags if ':=' in flag)
        enabled = {node for node in _APPLICATION_NODES
                   if options.get(f'with_{node}', 'true').lower()
                   not in ('false', '0')}
        if requested & enabled:
            return pid
    return None


def _clock_status(ros2_binary):
    now = time.monotonic()
    with _CLOCK_LOCK:
        if (now - _CLOCK_CACHE['at'] < 3.0 and
                _CLOCK_CACHE['result'] is not None):
            return _CLOCK_CACHE['result']
        try:
            environment = os.environ.copy()
            if 'ROS_LOG_DIR' not in environment:
                log_dir = Path(tempfile.gettempdir()) / (
                    f'drone_inspetor_ros_logs_{os.getuid()}')
                log_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
                environment['ROS_LOG_DIR'] = str(log_dir)
            completed = subprocess.run(
                [ros2_binary, 'topic', 'list', '--no-daemon'],
                capture_output=True, text=True, timeout=2, check=False,
                env=environment,
            )
        except (OSError, subprocess.TimeoutExpired):
            result = (
                'unknown', 'Consulta ROS indisponível ou expirou.', False)
        else:
            if completed.returncode != 0:
                result = (
                    'unknown', 'Não foi possível consultar os tópicos ROS.',
                    False)
            elif '/clock' in completed.stdout.splitlines():
                result = ('running', '/clock publicado na rede ROS.', True)
            else:
                result = (
                    'not_detected', '/clock não encontrado na rede ROS.',
                    False)
        _CLOCK_CACHE.update(at=now, result=result)
        return result


def _item(state, detail, available):
    return {'state': state, 'detail': detail, 'available': available}


def _package_prefix():
    """Encontra o pacote no overlay efetivamente carregado neste terminal."""
    try:
        from ament_index_python.packages import PackageNotFoundError
        from ament_index_python.packages import get_package_prefix
    except ImportError:
        return None
    try:
        return get_package_prefix('drone_inspetor')
    except PackageNotFoundError:
        return None


def environment_status():
    """Retorna estados; a consulta de /clock dura até dois segundos."""
    processes = _processes()
    names = [_executable(args) for _, args in processes]
    ros2_binary = shutil.which('ros2')
    package_prefix = _package_prefix()

    def process_item(present, found_text, missing_text):
        return _item('running' if present else 'not_detected',
                     found_text if present else missing_text, present)

    status = {
        'ros2': _item('available' if ros2_binary else 'unavailable',
                      ros2_binary or 'Comando ros2 não encontrado no PATH.',
                      bool(ros2_binary)),
        'package': _item(
            'available' if package_prefix else 'unavailable',
            package_prefix or 'Pacote drone_inspetor fora do overlay ROS 2.',
            bool(package_prefix)),
        'gazebo': process_item(
            any(name.startswith(('gz-sim', 'ign-gazebo')) or
                (name == 'gz' and 'sim' in args)
                for name, (_, args) in zip(names, processes)),
            'Servidor Gazebo detectado.', 'Servidor Gazebo não detectado.'),
        'px4': process_item('px4' in names,
                            'PX4 detectado.', 'PX4 não detectado.'),
        'micro_xrce_agent': process_item(
            any(name.startswith('microxrceagent') for name in names),
            'MicroXRCEAgent detectado.', 'MicroXRCEAgent não detectado.'),
        'qgroundcontrol': process_item(
            any(name.startswith('qgroundcontrol') for name in names),
            'QGroundControl detectado.', 'QGroundControl não detectado.'),
        'bridges': process_item(
            _bridge_processes(processes),
            'Pelo menos uma bridge ROS/Gazebo detectada.',
            'Bridges ROS/Gazebo não detectadas.'),
    }
    if ros2_binary:
        status['clock'] = _item(*_clock_status(ros2_binary))
    else:
        status['clock'] = _item('unavailable', 'ROS 2 indisponível.', False)
    registry = _Registry()
    with registry.locked():
        active = registry.read()
    if active:
        status['application'] = _item(
            'running',
            f'Modo {active["mode"]} gerenciado (PID {active["pid"]}).',
            True)
    else:
        external = _external_launch_conflict('sim', processes)
        status['application'] = process_item(
            bool(external), 'Aplicação ROS externa detectada.',
            'Aplicação ROS não detectada.')
    return status


def _configuration_file(value, name):
    """Valida arquivos fornecidos e torna caminhos relativos inequívocos."""
    if value is None or value == '':
        return None
    try:
        path = Path(value).expanduser()
        # Catálogos também podem usar o nome relativo documentado pelo launch.
        if name == 'missions_file' and not path.is_absolute() and not path.is_file():
            prefix = _package_prefix()
            if prefix:
                path = Path(prefix) / 'share/drone_inspetor/missions' / path
        path = path.resolve(strict=True)
        if not path.is_file():
            raise ValueError('o caminho não é um arquivo')
        with path.open('rb') as handle:
            handle.read(1)
    except (OSError, TypeError, ValueError, RuntimeError) as exc:
        raise ValueError(f'{name}: arquivo inválido ou ilegível: {value}') from exc
    return str(path)


def build_command(mode, *, bridges=None, use_sim_time=None, params_file=None,
                  missions_file=None, bridges_file=None, **node_options):
    """Monta um comando ros2 launch, sem shell e sem executá-lo."""
    selected = _selected_nodes(mode, node_options)
    if bridges is not None and not isinstance(bridges, bool):
        raise ValueError('bridges deve ser None, True ou False')
    if use_sim_time is not None and not isinstance(use_sim_time, bool):
        raise ValueError('use_sim_time deve ser None, True ou False')
    if mode != 'sim' and bridges is True:
        raise ValueError('Bridges só podem ser iniciadas pelo modo sim.')

    if mode == 'sim':
        enable_bridges = (not _bridge_processes(_processes())
                          if bridges is None else bridges)
        sim_clock = True if use_sim_time is None else use_sim_time
    elif mode == 'companion':
        enable_bridges = False
        sim_clock = False if use_sim_time is None else use_sim_time
    else:
        enable_bridges = False
        if use_sim_time is None:
            ros2_binary = shutil.which('ros2')
            sim_clock = bool(ros2_binary and _clock_status(ros2_binary)[2])
        else:
            sim_clock = use_sim_time

    flags = [f'bridges:={str(enable_bridges).lower()}',
             f'use_sim_time:={str(sim_clock).lower()}']
    for node, enabled in selected.items():
        flags.append(f'with_{node}:={str(enabled).lower()}')
    for name, value in (('params_file', params_file),
                        ('missions_file', missions_file),
                        ('bridges_file', bridges_file)):
        path = _configuration_file(value, name)
        if path is not None:
            flags.append(f'{name}:={path}')
    return ['ros2', 'launch', 'drone_inspetor', MODE_SPECS[mode].launch_file,
            *flags]


def _start_ticks(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
        return stat.rsplit(') ', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def _record_alive(record):
    try:
        pid = int(record['pid'])
        command = record['command']
        if _start_ticks(pid) != record['start_ticks']:
            return False
        raw = Path(f'/proc/{pid}/cmdline').read_bytes()
        actual = [part.decode('utf-8', 'replace')
                  for part in raw.split(b'\0') if part]
        return any(Path(actual[index]).name == Path(command[0]).name and
                   actual[index + 1:index + len(command)] == command[1:]
                   for index in range(len(actual) - len(command) + 1))
    except (OSError, ValueError, KeyError, TypeError):
        return False


class _Registry:
    """Uma sessão por usuário, protegida contra partidas simultâneas."""

    def __init__(self, directory=None):
        self.directory = Path(directory or Path(tempfile.gettempdir()) /
                              f'drone_inspetor_start_{os.getuid()}')
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.lock_path = self.directory / 'session.lock'
        self.state_path = self.directory / 'session.json'

    @contextmanager
    def locked(self):
        with self.lock_path.open('a+') as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle, fcntl.LOCK_UN)

    def read(self):
        try:
            record = json.loads(self.state_path.read_text())
        except (OSError, json.JSONDecodeError):
            return None
        if _record_alive(record):
            return record
        self.clear()
        return None

    def write(self, record):
        temporary = self.directory / f'.session.{os.getpid()}.json'
        temporary.write_text(json.dumps(record))
        temporary.replace(self.state_path)

    def clear(self):
        self.state_path.unlink(missing_ok=True)


class StartupController:
    """Controla uma sessão ros2 launch; a GUI pode consultar sem bloquear."""

    def __init__(self, runtime_dir=None):
        """Configura o registro compartilhado e o buffer de logs."""
        self._registry = _Registry(runtime_dir)
        self._process = None
        self._mode = None
        self._returncode = None
        self._reported_exit = False
        self._logs = deque(maxlen=1000)
        self._logs_lock = threading.Lock()
        self._stopping = False
        self._stop_thread = None

    @property
    def mode(self):
        """Modo iniciado por esta instância, enquanto estiver ativo."""
        if self._process is not None and self._process.poll() is None:
            return self._mode
        return None

    @property
    def running(self):
        """Indica se o processo desta instância ainda está ativo."""
        return self.mode is not None

    def active_session(self):
        """Lê uma sessão de qualquer interface, sem assumir posse."""
        with self._registry.locked():
            return self._registry.read()

    def start(self, mode, *, bridges=None, use_sim_time=None, params_file=None,
              missions_file=None, bridges_file=None, **node_options):
        """Inicia um launcher e retorna seus argumentos."""
        if not shutil.which('ros2'):
            raise RuntimeError(
                'Comando ros2 não encontrado. Carregue o ambiente ROS 2.')
        if not _package_prefix():
            raise RuntimeError(
                'Pacote drone_inspetor não encontrado no overlay ROS 2. '
                'Carregue install/setup.bash.')
        command = build_command(
            mode, bridges=bridges, use_sim_time=use_sim_time,
            params_file=params_file, missions_file=missions_file,
            bridges_file=bridges_file, **node_options)
        with self._registry.locked():
            record = self._registry.read()
            if record:
                raise RuntimeError(
                    f'Modo {record["mode"]} já ativo (PID {record["pid"]}).')
            processes = _processes()
            conflict = _external_launch_conflict(
                mode, processes, nodes=_selected_nodes(mode, node_options))
            if conflict:
                raise RuntimeError(
                    f'Aplicação ROS externa detectada (PID {conflict}).')
            if (mode == 'sim' and bridges is True and
                    _bridge_processes(processes)):
                raise RuntimeError(
                    'Bridges já detectadas. Use automático ou bridges=false.')
            if (mode == 'sim' and bridges is None and
                    _bridge_processes(processes)):
                command[4] = 'bridges:=false'
            try:
                process = subprocess.Popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, start_new_session=True,
                )
            except OSError as exc:
                raise RuntimeError(
                    f'Não foi possível iniciar ros2 launch: {exc}') from exc
            ticks = _start_ticks(process.pid)
            if ticks is None:
                process.terminate()
                raise RuntimeError(
                    'Processo ros2 launch encerrou durante a partida.')
            self._registry.write({'pid': process.pid, 'start_ticks': ticks,
                                  'command': command, 'mode': mode})
        self._process = process
        self._mode = mode
        self._returncode = None
        self._reported_exit = False
        self._stopping = False
        threading.Thread(
            target=self._capture_logs, args=(process,), daemon=True).start()
        return command

    def _capture_logs(self, process):
        try:
            for line in process.stdout:
                with self._logs_lock:
                    self._logs.append(line.rstrip('\r\n'))
        except (OSError, ValueError):
            pass
        finally:
            if process.stdout:
                process.stdout.close()

    def recent_logs(self):
        """Devolve e remove apenas as linhas novas; seguro para um QTimer."""
        with self._logs_lock:
            lines = list(self._logs)
            self._logs.clear()
        return lines

    def poll(self):
        """Retorna None enquanto ativo ou o código de saída local."""
        if self._process is None:
            return self._returncode
        result = self._process.poll()
        if result is None:
            return None
        self._returncode = result
        with self._registry.locked():
            record = self._registry.read()
            if record and record['pid'] == self._process.pid:
                self._registry.clear()
        if not self._reported_exit:
            with self._logs_lock:
                self._logs.append(f'ros2 launch encerrado (código {result}).')
            self._reported_exit = True
        return result

    def stop(self, *, wait=False):
        """Encerra somente o processo iniciado por esta instância."""
        if self._stopping:
            self._join_stop(wait)
            return
        if self._process is None or self._process.poll() is not None:
            return
        with self._registry.locked():
            record = self._registry.read()
        if not record or record['pid'] != self._process.pid:
            return
        self._request_stop(record, wait)

    def stop_active(self, *, wait=False):
        """Encerra explicitamente a sessão gerenciada por outra interface."""
        if self._stopping:
            self._join_stop(wait)
            return
        with self._registry.locked():
            record = self._registry.read()
        if not record:
            return
        self._request_stop(record, wait)

    def _join_stop(self, wait):
        if wait and self._stop_thread is not None:
            self._stop_thread.join(timeout=6.0)

    def _request_stop(self, record, wait):
        self._stopping = True
        if wait:
            self._stop_session(record)
        else:
            self._stop_thread = threading.Thread(
                target=self._stop_session, args=(record,), daemon=True)
            self._stop_thread.start()

    def _stop_session(self, record):
        try:
            for sig, delay in ((signal.SIGINT, 3.0),
                               (signal.SIGTERM, 2.0),
                               (signal.SIGKILL, 0.5)):
                if not _record_alive(record):
                    break
                try:
                    os.killpg(int(record['pid']), sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + delay
                while _record_alive(record) and time.monotonic() < deadline:
                    time.sleep(0.05)
        finally:
            if (self._process is not None and
                    self._process.pid == record['pid']):
                self.poll()
            with self._registry.locked():
                current = self._registry.read()
                if (current and current['pid'] == record['pid'] and
                        not _record_alive(current)):
                    self._registry.clear()
            self._stopping = False
