"""Valida composição dos launchers e instalação sem iniciar os processos."""

from pathlib import Path
import runpy
from unittest.mock import patch
import xml.etree.ElementTree as ET

import pytest
import launch.logging
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from drone_inspetor.launch.composition import create_launch


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module', autouse=True)
def temporary_launch_logs(tmp_path_factory):
    """Os launchers não escrevem no diretório ROS pessoal durante testes."""
    launch.logging.launch_config.log_dir = str(tmp_path_factory.mktemp('launch-logs'))


def configured_launch(**kwargs):
    """Resolve somente declarações/condições; nenhum executável é iniciado."""
    with patch('drone_inspetor.launch.composition.get_package_share_directory',
               return_value=str(ROOT / 'drone_inspetor')):
        description = create_launch(**kwargs)
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return description, context


def active_nodes(description, context):
    return [node for node in description.entities
            if isinstance(node, Node) and node.condition.evaluate(context)]


def test_real_context_starts_all_application_nodes_without_bridges():
    description, context = configured_launch()
    assert context.launch_configurations['use_sim_time'] == 'false'
    assert len(active_nodes(description, context)) == 7


def test_simulation_context_can_run_only_bridges():
    description, context = configured_launch(simulation=True, bridges=True, application=False)
    assert context.launch_configurations['use_sim_time'] == 'true'
    assert len(active_nodes(description, context)) == 2


@pytest.mark.parametrize('override', [None, '/tmp/bridges_plataforma_uerj_27.yaml'])
def test_bridge_config_uses_default_or_launch_override(override):
    with patch('drone_inspetor.launch.composition.Node', wraps=Node) as node_factory:
        _, context = configured_launch(simulation=True, bridges=True, application=False)
    expected = str(ROOT / 'drone_inspetor' / 'config' / 'ros_gz_bridges.yaml')
    assert context.launch_configurations['bridges_file'] == expected
    if override is not None:
        context.launch_configurations['bridges_file'] = override
        expected = override
    bridge_call = next(call for call in node_factory.call_args_list
                       if call.kwargs['package'] == 'ros_gz_bridge')
    parameters = evaluate_parameters(
        context, normalize_parameters(bridge_call.kwargs['parameters']))
    assert parameters[0]['config_file'] == expected


def test_individual_nodes_can_be_disabled_without_editing_launch():
    description, context = configured_launch(simulation=True, bridges=True)
    assert len(active_nodes(description, context)) == 9
    context.launch_configurations['with_dashboard'] = 'false'
    context.launch_configurations['with_cv'] = 'false'
    assert len(active_nodes(description, context)) == 7


def test_setup_installs_resources_once_and_uses_explicit_entry_modules(monkeypatch):
    captured = {}
    monkeypatch.chdir(ROOT)
    with patch('setuptools.setup', side_effect=lambda **kwargs: captured.update(kwargs)):
        runpy.run_path(str(ROOT / 'setup.py'))
    assert captured['version'] == ET.parse(ROOT / 'package.xml').findtext('version') == '2.0.0'
    files = [file for _, files in captured['data_files'] for file in files]
    assert 'drone_inspetor/gui/map.html' in files
    assert 'drone_inspetor/gui/leaflet_local/leaflet.js' in files
    assert 'drone_inspetor/gui/utils.py' not in files
    assert 'ruckig==0.19.4' in captured['install_requires']
    assert all(any(module in entry for module in (
        '.scripts.', '_node.', '.startup.cli:main', '.gui.startup_window:main',
        '.mobile_gateway.server:main'))
               for entry in captured['entry_points']['console_scripts'])
