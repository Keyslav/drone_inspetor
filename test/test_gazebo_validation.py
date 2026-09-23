"""A verificação pré-voo não confunde yaw Gazebo ENU com yaw PX4 NED."""

import importlib.util
import math
from pathlib import Path

import pytest


@pytest.fixture
def validator(monkeypatch):
    tools = Path(__file__).parents[1] / 'tools'
    monkeypatch.syspath_prepend(str(tools))
    spec = importlib.util.spec_from_file_location('validate_gazebo', tools / 'validate_gazebo.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_groundtruth_identity_points_east_in_px4_frame(validator):
    assert validator.groundtruth_yaw_ned('pose { orientation { w: 1 } }') == pytest.approx(math.pi/2)
    north = 'pose { orientation { z: 0.7071067811865476 w: 0.7071067811865476 } }'
    assert validator.groundtruth_yaw_ned(north) == pytest.approx(0., abs=1e-12)


@pytest.mark.parametrize('text', ['missing', 'orientation {}', 'orientation { w: nan }',
                                  'orientation { w: 2 }'])
def test_unusable_groundtruth_cannot_authorize_flight(validator, text):
    with pytest.raises(ValueError):
        validator.groundtruth_yaw_ned(text)
