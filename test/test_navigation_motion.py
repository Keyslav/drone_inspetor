"""Invariantes cinemáticas; não requer ROS, autopilot ou GPU."""

import math

import pytest

from drone_inspetor.navigation.motion import (
    SegmentProfile, braking_distance, speed_for_clearance, terminal_speed_limit,
)


@pytest.mark.parametrize('length', [0.05, 0.5, 3., 30.])
@pytest.mark.parametrize('dt', [0.01, 0.02, 0.05])
def test_segment_reaches_target_with_consistent_derivatives(length, dt):
    profile = SegmentProfile(3., 1., 2., 2.)
    target = (length / math.sqrt(2), length / math.sqrt(2), -length / 2)
    profile.start((0., 0., 0.), target)
    position = (0., 0., 0.)
    prev_velocity = prev_accel = 0.
    peak_speed = 0.
    for _ in range(int(60 / dt)):
        position, velocity, acceleration = profile.step(dt, position)
        speed = math.sqrt(sum(x*x for x in velocity))
        assert speed <= 3. + 1e-6
        assert -2. - 1e-6 <= profile.accel <= 1. + 1e-6
        assert abs(profile.accel - prev_accel) <= 2. * dt + 1e-6
        assert abs(profile.velocity - prev_velocity) <= 2. * dt + 1e-6
        assert profile.position <= profile.length + 1e-6
        prev_velocity, prev_accel = profile.velocity, profile.accel
        peak_speed = max(peak_speed, speed)
        if profile.finished:
            break
    assert profile.finished
    assert position == pytest.approx(target)
    if length >= 30:
        assert peak_speed == pytest.approx(3.)


def test_obstacle_braking_is_continuous_and_does_not_report_arrival():
    p = SegmentProfile(3., 1., 2., 2.)
    p.start((0., 0., 0.), (100., 0., 0.))
    pos = (0., 0., 0.)
    for _ in range(250):
        pos, _, _ = p.step(.02, pos)
    origin = pos[0]
    expected = braking_distance(p.velocity, p.accel, 2., 2.)
    prev_a = p.accel
    for _ in range(400):
        pos, _, _ = p.step(.02, pos, 0.)
        assert abs(p.accel - prev_a) <= .04 + 1e-6
        assert not p.finished
        prev_a = p.accel
    assert p.stopped
    assert pos[0] - origin == pytest.approx(expected, abs=1e-6)
    stopped = pos
    for _ in range(50):
        pos, _, _ = p.step(.02, pos, 0.)
        assert pos == pytest.approx(stopped)
    for _ in range(100):
        pos, _, _ = p.step(.02, pos, .6)
        assert p.velocity <= .6 + 1e-6
    assert p.velocity == pytest.approx(.6)


def test_lagging_vehicle_does_not_restart_or_push_reference_past_goal():
    p = SegmentProfile(3., 1., 2., 2.)
    p.start((0., 0., 0.), (3., 0., 0.))
    for _ in range(1000):
        pos, vel, acc = p.step(.02, (0., 0., 0.), measured_velocity=(0., 0., 0.))
        assert pos[0] <= 3. + 1e-6
    assert p.reference_finished and not p.finished
    assert pos == pytest.approx((3., 0., 0.))
    assert vel == pytest.approx((0., 0., 0.))


@pytest.mark.parametrize('distance', [0., .1, 1., 3., 8., 30.])
def test_speed_limit_fits_braking_envelope(distance):
    cap = speed_for_clearance(distance, 3., 2., 2., .35)
    assert 0 <= cap <= 3.
    assert braking_distance(cap, 0., 2., 2., .35) <= distance + 1e-6


def test_positive_acceleration_and_latency_are_included():
    assert braking_distance(3., 1., 2., 2., .35) > braking_distance(3., 0., 2., 2.)
    assert braking_distance(3., 0., 2., 2.) == pytest.approx(3.75)


@pytest.mark.parametrize('measured', [(3., 0., 0.), (3.5, 0., 0.)])
def test_terminal_floor_finishes_reference_with_vehicle_at_or_past_target(measured):
    profile = SegmentProfile(3., 1., 2., 2.)
    profile.start((0., 0., 0.), (3., 0., 0.))
    previous_acceleration = 0.
    for _ in range(600):
        cap = terminal_speed_limit(3. - measured[0], 3., 2., 2., .35, .6, 2.)
        assert cap == pytest.approx(.6)
        position, _, _ = profile.step(.02, measured, cap, (0., 0., 0.))
        assert abs(profile.accel - previous_acceleration) <= .040001
        previous_acceleration = profile.accel
        if profile.reference_finished:
            break
    assert profile.reference_finished and profile.stopped
    assert position == pytest.approx((3., 0., 0.))
    # Acabar a referência não autoriza chegada de um veículo ainda fora do alvo.
    assert profile.finished == (measured[0] == 3.)


def test_terminal_limit_keeps_cruise_far_away_and_reserves_slow_approach():
    assert terminal_speed_limit(40., 3., 2., 2., .35, .6, 2.) == pytest.approx(3., abs=1e-6)
    assert terminal_speed_limit(1.5, 3., 2., 2., .35, .6, 2.) == pytest.approx(.6)


@pytest.mark.parametrize('value', [0., -1., math.nan, math.inf])
def test_invalid_limits_rejected(value):
    with pytest.raises(ValueError):
        SegmentProfile(value, 1., 2., 2.)
