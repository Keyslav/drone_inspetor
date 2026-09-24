"""O diagnóstico de instalação interrompe o launch antes de iniciar nós."""

from unittest.mock import patch
import pytest

from drone_inspetor.launch.preflight import check_runtime, check_launch


def test_old_interfaces_explain_rebuild():
    with patch('drone_inspetor.launch.preflight.import_module',
               side_effect=ImportError('DashboardMissionCommandMSG')):
        with pytest.raises(RuntimeError, match='colcon build --symlink-install'):
            check_runtime()


def test_missing_ruckig_explains_python_environment():
    with patch('drone_inspetor.launch.preflight.import_module',
               side_effect=[object(), ImportError('ruckig')]):
        with pytest.raises(RuntimeError, match='mesmo ambiente'):
            check_runtime(drone=True)


def test_bridges_only_need_no_application_dependencies(tmp_path, monkeypatch):
    from launch import LaunchContext
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config, 'log_dir', str(tmp_path))
    context = LaunchContext()
    context.launch_configurations['with_drone'] = 'false'
    with patch('drone_inspetor.launch.preflight.check_runtime') as check:
        assert check_launch(context, ['drone']) == []
        check.assert_not_called()


def test_installed_runtime_is_compatible():
    check_runtime(drone=True)
