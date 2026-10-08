"""Comandos e posse dos processos do inicializador."""

import os
import time

from drone_inspetor.startup import StartupController, build_command
from drone_inspetor.startup import cli
from drone_inspetor.startup import controller as startup

import pytest


def test_modes_select_exactly_the_expected_application_nodes(monkeypatch):
    """Cada perfil seleciona exatamente seus nós e seu relógio."""
    monkeypatch.setattr(startup, '_processes', lambda: [])
    monkeypatch.setattr(startup.shutil, 'which', lambda name: '/usr/bin/ros2')
    monkeypatch.setattr(
        startup, '_clock_status', lambda _: ('running', '/clock', True))

    sim = build_command('sim')
    assert sim[:4] == [
        'ros2', 'launch', 'drone_inspetor', 'dashboard_launch.py']
    assert 'bridges:=true' in sim and 'use_sim_time:=true' in sim
    assert all(f'with_{node}:=true' in sim
               for node in startup._APPLICATION_NODES)

    companion = build_command('companion')
    assert companion[3] == 'drone_inspetor_launch.py'
    assert 'bridges:=false' in companion and 'use_sim_time:=false' in companion
    assert 'with_dashboard:=false' in companion
    assert all(f'with_{node}:=true' in companion
               for node in startup._APPLICATION_NODES if node != 'dashboard')

    dashboard = build_command('dashboard')
    assert 'with_dashboard:=true' in dashboard
    assert 'use_sim_time:=true' in dashboard
    assert 'bridges:=false' in dashboard
    assert all(f'with_{node}:=false' in dashboard
               for node in startup._APPLICATION_NODES if node != 'dashboard')


def test_auto_bridges_uses_existing_bridge_process(monkeypatch):
    """A seleção automática reconhece bridge pré-existente."""
    bridge = '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
    monkeypatch.setattr(startup, '_processes', lambda: [(10, [bridge])])
    assert 'bridges:=false' in build_command('sim')
    assert 'bridges:=true' in build_command('sim', bridges=True)
    with pytest.raises(ValueError, match='Bridges só'):
        build_command('dashboard', bridges=True)


def test_custom_files_remain_single_arguments_and_nodes_override_defaults(
        tmp_path, monkeypatch):
    """Caminhos com espaços/metacaracteres nunca são interpretados por shell."""
    monkeypatch.chdir(tmp_path)
    config = tmp_path / 'param $(touch unexpected).yaml'
    config.write_text('{}')
    missions = tmp_path / 'catalogo local.json'
    missions.write_text('[]')
    bridges = tmp_path / 'bridges.yaml'
    bridges.write_text('[]')
    command = build_command(
        'sim', bridges=False, params_file=config.name,
        missions_file=missions, bridges_file=bridges, with_cv=False,
        with_camera=False, with_drone=True)
    assert f'params_file:={config}' in command
    assert f'missions_file:={missions}' in command
    assert f'bridges_file:={bridges}' in command
    assert 'with_cv:=false' in command
    assert 'with_camera:=false' in command
    assert 'with_drone:=true' in command
    assert not (tmp_path / 'unexpected').exists()
    assert 'with_cv:=true' in build_command('companion')
    assert 'with_cv:=true' in build_command('sim', bridges=False)


@pytest.mark.parametrize('name', ['params_file', 'missions_file', 'bridges_file'])
def test_custom_files_reject_missing_paths_and_directories(name, tmp_path):
    """Erros locais são mostrados antes da execução de ros2 launch."""
    for value in (tmp_path / 'missing.yaml', tmp_path):
        with pytest.raises(ValueError, match=name):
            build_command('companion', **{name: value})


def test_catalog_name_resolves_from_installed_missions(tmp_path, monkeypatch):
    """Nomes de catálogos instalados preservam o contrato do launch."""
    catalog = tmp_path / 'share/drone_inspetor/missions/inspecao.json'
    catalog.parent.mkdir(parents=True)
    catalog.write_text('[]')
    monkeypatch.setattr(startup, '_package_prefix', lambda: str(tmp_path))
    assert f'missions_file:={catalog}' in build_command(
        'companion', missions_file='inspecao.json')


@pytest.mark.parametrize('options', [
    {'with_drone': 'false'}, {'with_camara': True},
    {'with_cv': 0}, {'bridges': 1}, {'use_sim_time': 0},
])
def test_invalid_start_options_are_rejected(options):
    """Erros de digitação não habilitam nós ou relógios silenciosamente."""
    with pytest.raises(ValueError):
        build_command('companion', **options)


def test_conflicts_use_selected_nodes_and_last_launch_argument():
    """Nós desabilitados não causam falso conflito com processos externos."""
    external = [(50, ['ros2', 'run', 'drone_inspetor', 'cv_node'])]
    selected = startup._selected_nodes('companion', {'with_cv': False})
    assert startup._external_launch_conflict(
        'companion', external, nodes=selected) is None
    assert startup._external_launch_conflict('companion', external) == 50
    flags = [f'with_{node}:=False' for node in startup._APPLICATION_NODES]
    launch = ['ros2', 'launch', 'drone_inspetor', 'dashboard_launch.py', *flags]
    assert startup._external_launch_conflict('sim', [(51, launch)]) is None
    launch.append('with_cv:=true')
    assert startup._external_launch_conflict('sim', [(51, launch)]) == 51
    assert startup._external_launch_conflict(
        'companion', [(51, launch)], nodes=selected) is None


def test_external_launch_conflicts_only_when_nodes_overlap():
    """Um dashboard externo conflita com sim, mas não com companion."""
    external = ['python3', '/opt/ros/jazzy/bin/ros2', 'launch',
                'drone_inspetor',
                'dashboard_launch.py', 'with_camera:=false', 'with_cv:=false',
                'with_depth:=false', 'with_lidar:=false', 'with_drone:=false',
                'with_mission:=false', 'with_dashboard:=true']
    processes = [(321, external)]
    assert startup._external_launch_conflict('sim', processes) == 321
    assert startup._external_launch_conflict('dashboard', processes) == 321
    assert startup._external_launch_conflict('companion', processes) is None


def test_external_ros2_run_and_direct_node_are_detected():
    """Nós individuais em execução também impedem duplicatas."""
    dashboard_run = [(456, ['python3', '/opt/ros/jazzy/bin/ros2', 'run',
                            'drone_inspetor', 'dashboard_node'])]
    drone_direct = [(789, ['python3',
                           '/home/user/ros2_ws/install/drone_inspetor/lib/'
                           'drone_inspetor/drone_node', '--ros-args'])]
    assert startup._external_launch_conflict(
        'dashboard', dashboard_run) == 456
    assert startup._external_launch_conflict(
        'companion', dashboard_run) is None
    assert startup._external_launch_conflict('companion', drone_direct) == 789
    assert startup._external_launch_conflict('dashboard', drone_direct) is None


def test_shebang_launch_can_be_tracked_and_stopped_from_another_controller(
        tmp_path, monkeypatch):
    """O registro acompanha ros2 mesmo quando ele roda via Python shebang."""
    ros2 = tmp_path / 'ros2'
    ros2.write_text(
        '#!/usr/bin/env python3\n'
        'import signal, sys, time\n'
        'signal.signal(signal.SIGINT, lambda *_: sys.exit(0))\n'
        'print("ready", flush=True)\n'
        'while True: time.sleep(0.05)\n')
    ros2.chmod(0o755)
    monkeypatch.setenv(
        'PATH', str(tmp_path) + os.pathsep + os.environ.get('PATH', ''))
    monkeypatch.setattr(startup, '_package_prefix', lambda: str(tmp_path))
    monkeypatch.setattr(startup, '_processes', lambda: [
        (999999, ['ros2', 'run', 'drone_inspetor', 'cv_node'])])
    runtime = tmp_path / 'runtime'
    owner = StartupController(runtime_dir=runtime)
    observer = StartupController(runtime_dir=runtime)
    try:
        owner.start('companion', with_cv=False)
        deadline = time.monotonic() + 3
        while ('ready' not in owner.recent_logs() and
               time.monotonic() < deadline):
            time.sleep(0.01)
        assert owner.running
        assert observer.active_session()['mode'] == 'companion'
        assert not observer.running
        observer.stop()  # A janela que não iniciou o processo não o encerra.
        assert owner.running
        with pytest.raises(RuntimeError, match='já ativo'):
            observer.start('sim')
        observer.stop_active()
        observer.stop_active(wait=True)
        assert owner.poll() == 0
        assert observer.active_session() is None
    finally:
        if owner.running:
            owner.stop(wait=True)


def test_start_reports_unsourced_application_overlay(tmp_path, monkeypatch):
    """A ausência de overlay é informada antes de criar o subprocesso."""
    monkeypatch.setattr(startup.shutil, 'which', lambda _: '/usr/bin/ros2')
    monkeypatch.setattr(startup, '_package_prefix', lambda: None)
    controller = StartupController(runtime_dir=tmp_path)
    with pytest.raises(RuntimeError, match='install/setup.bash'):
        controller.start('companion')


def test_terminal_menu_routes_start_status_logs_stop_and_exit(
        monkeypatch, capsys):
    """O menu oferece os comandos centrais sem iniciar ROS no teste."""
    events = []
    answers = iter(('1', '4', '5', '6', '0'))
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    monkeypatch.setattr(cli, '_print_status',
                        lambda: events.append('status'))

    class Controller:
        running = False

        def poll(self):
            return None

        def active_session(self):
            return {'mode': 'sim', 'pid': 42} if self.running else None

        def start(self, mode):
            events.append(('start', mode))
            self.running = True
            return ['ros2', 'launch']

        def recent_logs(self):
            events.append('logs')
            return ['linha de teste']

        def stop_active(self, *, wait=False):
            events.append(('stop_active', wait))
            self.running = False

        def stop(self, *, wait=False):
            events.append(('stop', wait))
            self.running = False

    assert cli._interactive_menu(Controller()) == 0
    assert ('start', 'sim') in events
    assert 'status' in events
    assert ('stop_active', True) in events
    assert ('stop', True) not in events
    assert 'linha de teste' in capsys.readouterr().out


def test_cli_forwards_file_and_individual_node_options(monkeypatch):
    """Flags explícitas chegam ao controlador; nós omitidos usam o perfil."""
    calls = []

    class Controller:
        def start(self, mode, **options):
            calls.append((mode, options))
            return ['ros2', 'launch']

        def recent_logs(self):
            return []

        def poll(self):
            return 0

    monkeypatch.setattr(cli, 'StartupController', Controller)
    assert cli.main([
        'start', 'companion', '--params-file', '/tmp/params file.yaml',
        '--missions-file', '/tmp/missions.json', '--no-with-cv',
        '--with-dashboard', '--time', 'real',
    ]) == 0
    mode, options = calls[0]
    assert mode == 'companion'
    assert options['params_file'] == '/tmp/params file.yaml'
    assert options['missions_file'] == '/tmp/missions.json'
    assert options['with_cv'] is False
    assert options['with_dashboard'] is True
    assert options['with_drone'] is None
    assert options['use_sim_time'] is False
