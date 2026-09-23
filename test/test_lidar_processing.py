"""Parsing, direção e snapshots de flags do LiDAR legado."""

import math

import pytest

from drone_inspetor.nodes.lidar_node.processing import (
    ground_distance, point_vector, scan_points, sector_flags,
)


def test_angles_use_increment_and_invalid_values_do_not_become_points():
    ranges, angles = scan_points([1., math.nan, 2., math.inf, -math.inf, 0., -1., 12.],
                                -.7, .2, .1, 10.)
    assert point_vector(ranges, angles) == pytest.approx([1., -.7, 2., -.3])


def test_flu_left_and_right_are_not_reversed():
    left = sector_flags([.8], [math.pi / 2])
    right = sector_flags([.8], [-math.pi / 2])
    assert left['have_obstacles_left_90'] and not left['have_obstacles_right_90']
    assert right['have_obstacles_right_90'] and not right['have_obstacles_left_90']


def test_sector_flags_clear_on_next_scan_without_stale_cooldown():
    near = sector_flags([.5], [0.])
    clear = sector_flags([10.], [0.])
    assert near['have_obstacles_front_90'] and near['have_obstacles_1m']
    assert not any(clear.values())


def test_boundaries_belong_to_one_sector_and_angles_wrap():
    for angle in (math.pi / 4, 3 * math.pi / 4, -math.pi / 4, -3 * math.pi / 4, 7 * math.pi):
        flags = sector_flags([.5], [angle])
        assert sum(value for name, value in flags.items() if name.endswith('_90')) == 1


def test_ground_invalid_is_unknown_and_near_saturation_is_not_dropped():
    assert math.isnan(ground_distance([math.inf, math.nan, -1], .1, 10.))
    assert ground_distance([.05, .4], .1, 10.) == .1
