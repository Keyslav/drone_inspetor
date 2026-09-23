"""O relatório de ensaio não confunde picos ou lacunas com cruzeiro sustentado."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

_spec = importlib.util.spec_from_file_location(
    'validate_sitl', Path(__file__).parents[1] / 'tools' / 'validate_sitl.py')
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)


def sample(time, speed=3., reference_speed=3.):
    return {'time': time, 'velocity': (speed, 0., 0.), 'reference_velocity': reference_speed,
            'position': (0., 0., 0.), 'reference': (.2, 0., 0.),
            'phase': 'DESLOCANDO', 'obstacles': [(2., 0.)]}


def report(samples):
    return _module.flight_metrics({
        'commands': [{'command': 'GOTO', 'started': 0., 'finished': 10.}],
        'samples': samples,
    }, 3.)[0]


def test_cruise_requires_continuous_measured_and_commanded_speed():
    samples = [sample(i * .05) for i in range(50)]
    assert report(samples)['continuous_cruise_s'] == pytest.approx(2.45)
    samples[20]['velocity'] = (1., 0., 0.)
    assert report(samples)['continuous_cruise_s'] == pytest.approx(1.4)
    samples[35]['reference_velocity'] = 1.
    assert report(samples)['continuous_cruise_s'] == pytest.approx(.95)


def test_missing_samples_do_not_prove_cruise_and_geometry_uses_surface():
    result = report([sample(0.), sample(.1), sample(4.), sample(4.1)])
    assert result['continuous_cruise_s'] == pytest.approx(.1)
    assert result['maximum_tracking_error_m'] == pytest.approx(.2)
    assert result['minimum_obstacle_surface_distance_m'] == pytest.approx(1.6)


def test_native_mode_does_not_compare_against_inactive_ros_reference():
    samples = [sample(0.), sample(.05)]
    for item in samples:
        item['reference_active'] = False
        item['reference'] = (0., 0., -3.)
    result = report(samples)
    assert result['maximum_tracking_error_m'] is None
    assert result['peak_reference_speed_m_s'] is None
    assert result['continuous_cruise_s'] == 0.
    assert result['peak_measured_speed_m_s'] == 3.


def test_transport_timeout_preserves_failed_command_metrics(monkeypatch):
    times = iter((0., 5.))
    monkeypatch.setattr(_module.time, 'monotonic', lambda: next(times))
    data = {'commands': [], 'samples': [sample(1.), sample(1.05)]}

    def timeout():
        raise TimeoutError('Sem resposta ROS')

    with pytest.raises(TimeoutError, match='Sem resposta ROS'):
        _module.record_command(data, 'GOTO', {'lat': 1.}, timeout)

    assert len(data['commands']) == 1
    assert data['commands'][0]['success'] is False
    assert data['commands'][0]['error_type'] == 'TimeoutError'
    metric = _module.flight_metrics(data, 3.)[0]
    assert metric['duration_s'] == 5.
    assert metric['minimum_obstacle_surface_distance_m'] == pytest.approx(1.6)


def test_rejected_result_is_recorded_without_turning_it_into_success():
    data = {'commands': []}
    result = SimpleNamespace(success=False, message='Frenagem não confirmada')
    assert _module.record_command(data, 'GOTO', {}, lambda: result) is result
    assert data['commands'][0]['success'] is False
    assert data['commands'][0]['message'] == result.message
    assert data['commands'][0]['finished'] >= data['commands'][0]['started']
