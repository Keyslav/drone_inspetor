"""Geometria, validade dos sensores e escolhas do planejador local."""

import math

import pytest

from drone_inspetor.navigation.obstacles import ObstacleMap, LocalPlanner


def scan(obstacles, position=(0., 0., 0.), yaw=0., now=1., radius=.4):
    ranges = []
    for i in range(720):
        angle = yaw - (-math.pi + i * math.pi / 360)
        ux, uy = math.cos(angle), math.sin(angle)
        distance = math.inf
        for x, y in obstacles:
            dx, dy = x-position[0], y-position[1]
            along = dx*ux + dy*uy
            cross = abs(dx*uy-dy*ux)
            if along > 0 and cross < radius:
                distance = min(distance, max(0.1, along - math.sqrt(radius**2-cross**2)))
        ranges.append(distance)
    return ranges


def update(field, points=(), position=(0., 0., 0.), yaw=0., now=1.):
    field.update('lidar', scan(points, position, yaw), -math.pi, math.pi/360,
                 .1, 15., position, yaw, now)


@pytest.mark.parametrize('yaw', [0., .5, math.pi/2, math.pi, -2.])
def test_clearance_is_independent_of_vehicle_yaw(yaw):
    field = ObstacleMap(radius=.8)
    update(field, [(5., 0.)], yaw=yaw)
    assert field.clearance((0., 0., 0.), 0., 8., 1.) == pytest.approx(3.8, abs=.08)
    assert field.clearance((0., 0., 0.), math.pi/2, 8., 1.) > 7.


def test_no_sensor_invalid_beams_and_stale_data_stop_motion():
    field = ObstacleMap()
    planner = LocalPlanner()
    args = (field, (0., 0., 0.), (10., 0., 0.), 0., 0.)
    assert planner.evaluate(*args, 1.).reason == 'sensor_stale'
    update(field)
    assert planner.evaluate(*args, 1.).speed_limit > 0
    assert planner.evaluate(*args, 2.).speed_limit == 0
    field.update('lidar', [math.nan]*720, -math.pi, math.pi/360,
                 .1, 15., (0., 0., 0.), 0., 3.)
    assert planner.evaluate(*args, 3.).speed_limit == 0


def test_brakes_before_changing_route_then_selects_observed_corridor():
    field = ObstacleMap()
    update(field, [(5., 0.)])
    planner = LocalPlanner()
    decision = planner.evaluate(field, (0., 0., 0.), (12., 0., 0.), 3., 0., 1., 3.)
    assert decision.reason == 'braking_for_detour'
    assert decision.speed_limit == 0 and decision.detour is None
    decision = planner.evaluate(field, (0., 0., 0.), (12., 0., 0.), 0., 0., 1.)
    assert decision.reason == 'detour' and decision.detour is not None
    bearing = math.atan2(decision.detour[1], decision.detour[0])
    assert field.clearance((0., 0., 0.), bearing, 3., 1.) == pytest.approx(3.)


def test_detour_prefers_observed_exit_avoiding_an_immediate_second_stop():
    field = ObstacleMap(radius=.8)
    update(field, [(6., 0.)])
    planner = LocalPlanner()
    target = (12., 0., 0.)
    decision = planner.evaluate(field, (0., 0., 0.), target, 0., 0., 1.)
    point = decision.detour
    assert point is not None
    bearing = math.atan2(-point[1], target[0] - point[0])
    assert field.clearance(point, bearing, 6., 1.) >= 5.9
    # Depois de alcançar o candidato, novas observações permitem retomar a rota.
    update(field, [(6., 0.)], position=point, now=2.)
    resumed = planner.evaluate(field, point, target, 0., 0., 2.)
    assert resumed.detour is None and resumed.speed_limit > 0.


def test_partially_hidden_exit_still_allows_an_observed_progressive_detour():
    field = ObstacleMap(radius=1.3)
    update(field, [(6., 0.)])
    planner = LocalPlanner()
    target = (12., 0., 0.)
    point = planner.evaluate(field, (0., 0., 0.), target, 0., 0., 1.).detour
    assert point is not None
    first_heading = math.atan2(point[1], point[0])
    assert field.clearance((0., 0., 0.), first_heading, 3., 1.) == pytest.approx(3.)
    exit_heading = math.atan2(-point[1], target[0] - point[0])
    assert 0. < field.clearance(point, exit_heading, 6., 1.) < 6.


def test_obstacle_behind_does_not_force_braking_in_clear_forward_corridor():
    field = ObstacleMap()
    update(field, [(-3., 0.)])
    decision = LocalPlanner().evaluate(field, (0., 0., 0.), (12., 0., 0.), 1., 0., 1.)
    assert decision.speed_limit > 0 and decision.detour is None


def test_surrounded_vehicle_waits_without_arbitrary_climb():
    field = ObstacleMap()
    update(field, [(1.2*math.cos(a), 1.2*math.sin(a)) for a in [i*math.pi/8 for i in range(16)]])
    decision = LocalPlanner().evaluate(field, (0., 0., -2.), (12., 0., -2.), 0., 0., 1.)
    assert decision.reason == 'blocked' and decision.detour is None


def metric_scan(field, values, angle_min=-math.pi, step=math.pi / 180,
                origin=(0., 0., 0.), max_range=30.):
    field.update('lidar', values, angle_min, step, .1, max_range, origin, 0., 1.)


@pytest.mark.parametrize('angle', [10, 20, 30, 45])
def test_every_unknown_angular_cell_limits_swept_corridor(angle):
    field = ObstacleMap(radius=.8)
    values = [math.inf] * 361
    values[180 + angle] = math.nan
    metric_scan(field, values)
    assert field.clearance((0., 0., 0.), 0., 6., 1.) == 0.


@pytest.mark.parametrize('value', [.05, -math.inf])
def test_below_minimum_is_near_hit_not_a_discarded_ray(value):
    field = ObstacleMap(radius=.8)
    values = [math.inf] * 361
    values[225] = value
    metric_scan(field, values)
    assert len(field.scans['lidar'].hits) == 1
    assert field.clearance((0., 0., 0.), 0., 6., 1.) == 0.


@pytest.mark.parametrize('values,step', [([], .1), ([math.inf], 0.), ([math.inf], math.nan)])
def test_structurally_invalid_scan_removes_previous_free_observation(values, step):
    field = ObstacleMap()
    update(field)
    metric_scan(field, values, step=step)
    assert not field.fresh(1.)
    assert field.clearance((0., 0., 0.), 0., 6., 1.) == 0.


@pytest.mark.parametrize('origin', [(0., 0., 0.), (.12, 0., 0.), (.12, -.1, 0.)])
def test_270_degree_fov_allows_forward_and_blocks_unobserved_reverse(origin):
    field = ObstacleMap(radius=.8)
    metric_scan(field, [math.inf] * 1080, angle_min=-3 * math.pi / 4,
                step=1.5 * math.pi / 1079, origin=origin)
    assert field.clearance((0., 0., 0.), 0., 6., 1.) == pytest.approx(6.)
    assert field.clearance((0., 0., 0.), math.pi, 6., 1.) == 0.


def test_maximum_range_includes_vehicle_radius_and_does_not_extrapolate():
    field = ObstacleMap(radius=.8)
    metric_scan(field, [math.inf] * 361, max_range=3.)
    assert field.clearance((0., 0., 0.), 0., 6., 1.) == pytest.approx(2.2)


def test_scan_translation_is_part_of_range_coverage():
    field = ObstacleMap(radius=.8)
    metric_scan(field, [math.inf] * 361, origin=(.2, 0., 0.), max_range=3.)
    assert field.clearance((0., 0., 0.), 0., 6., 1.) == pytest.approx(2.4)
    assert field.clearance((0., 0., 0.), math.pi, 6., 1.) == pytest.approx(2.)


def test_hit_behind_with_negative_chord_exit_does_not_block_forward_motion():
    from dataclasses import replace
    field = ObstacleMap(radius=.8)
    update(field)
    field.scans['lidar'] = replace(field.scans['lidar'], hits=((-0.6, 0.6),))
    assert field.clearance((0., 0., 0.), 0., 6., 1.) == pytest.approx(6.)


def test_unknown_cell_boundary_is_checked_between_its_central_rays():
    from drone_inspetor.navigation.obstacle_geometry import unknown_sector_entry
    # O rumo passa por dentro da célula, sem coincidir com raio/borda discretizados.
    assert unknown_sector_entry((0., 0.), -.4, .2, 0., .8, 6.) == 0.
    # Frente observada até 2m: contato interno ocorre em 2-.8, não na borda angular.
    assert unknown_sector_entry((0., 0.), -.4, .2, 2., .8, 6.) == pytest.approx(1.2)


def test_expired_optional_depth_does_not_suppress_primary_lidar_coverage():
    field = ObstacleMap(radius=.8)
    metric_scan(field, [math.inf] * 361)
    field.update('depth', [math.nan, math.nan], -.5, 1., .1, 10., (0., 0., 0.), 0., 0.)
    assert field.clearance((0., 0., 0.), 0., 6., 1.) == pytest.approx(6.)


def test_analytic_sector_bound_never_exceeds_independent_dense_point_oracle():
    import numpy as np
    from drone_inspetor.navigation.obstacle_geometry import unknown_sector_entry
    random = np.random.default_rng(164)
    for _ in range(40):
        origin = tuple(random.uniform(-1.2, 1.2, 2))
        start = float(random.uniform(-math.pi, math.pi))
        end = start + float(random.uniform(.01, 2 * math.pi))
        observed = float(random.uniform(0., 7.))
        exact = unknown_sector_entry(origin, start, end, observed, .8, 6.)
        angles = np.linspace(start, end, 201)
        radial = np.linspace(observed, math.hypot(*origin) + 6.8, 201)
        x = origin[0] + radial[:, None] * np.cos(angles)[None, :]
        y = origin[1] + radial[:, None] * np.sin(angles)[None, :]
        valid = (x * x + y * y > (.8 + 1e-7) ** 2) & (abs(y) <= .8)
        chord = np.sqrt(np.maximum(0., .8 ** 2 - y * y))
        valid &= x + chord >= 0.
        oracle = float(np.where(valid, np.maximum(0., x - chord), np.inf).min())
        if oracle <= 6.:
            assert exact <= oracle + 1e-7
