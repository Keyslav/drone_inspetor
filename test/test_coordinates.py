"""Contratos de eixos/origens: casos cardeais para evitar inversões silenciosas."""

import math

import pytest

from drone_inspetor.common.coordinates import (
    amsl_to_local_down, body_flu_offset_to_ned, enu_to_ned, flu_to_frd,
    frd_to_flu, ned_to_enu, scan_angle_to_ned, yaw_enu_to_ned, yaw_ned_to_enu,
)
from drone_inspetor.common.math_utils import global_to_local_offset, global_to_ned_offset


def test_world_and_body_frames_are_not_the_same_conversion():
    assert enu_to_ned((10., 20., 5.)) == (20., 10., -5.)
    assert ned_to_enu((20., 10., -5.)) == (10., 20., 5.)
    assert flu_to_frd((10., 20., 5.)) == (10., -20., -5.)
    assert frd_to_flu((10., -20., -5.)) == (10., 20., 5.)


@pytest.mark.parametrize('enu,ned', [(0., 90.), (90., 0.), (180., -90.), (-90., -180.)])
def test_cardinal_yaws(enu, ned):
    assert math.degrees(yaw_enu_to_ned(math.radians(enu))) == pytest.approx(ned)
    roundtrip = math.degrees(yaw_ned_to_enu(math.radians(ned)))
    assert (roundtrip - enu + 180) % 360 - 180 == pytest.approx(0.)


@pytest.mark.parametrize('yaw,expected', [(0., (0., -1., 0.)),
                                          (90., (1., 0., 0.)),
                                          (180., (0., 1., 0.))])
def test_left_sensor_offset_and_left_scan_agree_in_world(yaw, expected):
    yaw = math.radians(yaw)
    offset = body_flu_offset_to_ned((0., 1., 0.), yaw)
    assert offset == pytest.approx(expected, abs=1e-12)
    bearing = scan_angle_to_ned(math.pi / 2, yaw)
    assert (math.cos(bearing), math.sin(bearing), 0.) == pytest.approx(expected, abs=1e-12)


def test_sensor_mount_rotation_is_applied_once():
    # Drone rumo Leste, sensor montado 90° à esquerda: raio frontal aponta Norte.
    assert scan_angle_to_ned(0., math.pi / 2, math.pi / 2) == pytest.approx(0.)


def test_amsl_home_and_local_origin_are_distinct():
    # HOME em 500m AMSL tem z=-7m; destino 510m AMSL fica em z=-17m.
    assert amsl_to_local_down(510., 500., -7.) == -17.
    assert global_to_ned_offset(-22., -43., 500., -22., -43., 510.) == (0., 0., -10.)
    # API antiga conserva seu resultado, mas não será mais usada como NED.
    assert global_to_local_offset(-22., -43., 500., -22., -43., 510.) == (0., 0., 10.)
