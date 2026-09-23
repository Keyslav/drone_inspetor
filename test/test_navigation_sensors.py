"""Idade de aquisição, interpolação de pose e separação de frames."""

import math
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import LaserScan

from drone_inspetor.navigation.config import NavigationConfig
from drone_inspetor.navigation.poses import PoseHistory
from drone_inspetor.nodes.drone_node.navigation_sensors import NavigationSensors


def test_pose_interpolates_yaw_through_wrap_and_rejects_gaps():
    history = PoseHistory()
    history.add(1., (0., 0., 0.), math.radians(179.))
    history.add(1.02, (2., 0., 0.), math.radians(-179.))
    pose = history.at(1.01)
    assert pose.position == pytest.approx((1., 0., 0.))
    assert abs(pose.yaw) == pytest.approx(math.pi)
    assert history.at(.5) is None
    assert history.at(2.) is None
    history.add(1.5, (4., 0., 0.), 0.)
    assert history.at(1.2) is None
    history.add(.1, (0., 0., 0.), 0.)
    assert len(history.samples) == 1


def adapter(monkeypatch):
    clock = NS(value=10.)
    monkeypatch.setattr('drone_inspetor.nodes.drone_node.navigation_sensors.time.monotonic', lambda: clock.value)
    monkeypatch.setattr('drone_inspetor.nodes.drone_node.navigation_sensors.create_subscription_from', lambda *args: None)
    history = PoseHistory()
    node = NS(state_px4=NS(local_position=NS(x=2., y=0., z=-3.), current_yaw_rad=0.),
              telemetry_fresh=lambda: True, pose_history=history,
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(clock.value * 1e9))))
    sensor = NavigationSensors(node, NavigationConfig())
    return sensor, node, clock


def scan(stamp, distance=5.):
    msg = LaserScan()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp % 1) * 1e9))
    msg.angle_min, msg.angle_increment = 0., .1
    msg.range_min, msg.range_max = .1, 30.
    msg.ranges = [distance]
    return msg


def test_delayed_scan_uses_capture_pose_and_does_not_get_new_lifetime(monkeypatch):
    sensor, node, clock = adapter(monkeypatch)
    node.pose_history.add(9.3, (0., 0., -3.), 0.)
    node.pose_history.add(10., (2., 0., -3.), math.pi / 2)
    sensor.lidar_callback(scan(9.3))
    reading = sensor.map.scans['lidar']
    assert reading.origin == (0., 0., -3.)
    assert reading.hits[0] == pytest.approx((5., 0.))
    assert reading.stamp == pytest.approx(9.3)
    assert sensor.map.fresh(10.)
    assert not sensor.map.fresh(10.1)


def test_duplicate_does_not_renew_reading_and_clock_reset_recovers(monkeypatch):
    sensor, node, clock = adapter(monkeypatch)
    node.pose_history.add(10., (2., 0., -3.), 0.)
    sensor.lidar_callback(scan(10.))
    clock.value = 10.1
    sensor.lidar_callback(scan(10.))
    assert sensor.map.scans['lidar'].stamp == 10.
    clock.value = 1.
    node.pose_history.add(1., (0., 0., -3.), 0.)
    sensor.lidar_callback(scan(1.))
    assert sensor.map.scans['lidar'].stamp == 1.


def test_scan_without_corresponding_pose_cannot_reuse_old_free_map(monkeypatch):
    sensor, node, clock = adapter(monkeypatch)
    node.pose_history.add(10., (2., 0., -3.), 0.)
    sensor.lidar_callback(scan(10.))
    clock.value = 10.5
    sensor.lidar_callback(scan(10.5))
    assert not sensor.map.fresh(clock.value)


def test_down_clearance_accounts_for_descent_since_acquisition(monkeypatch):
    sensor, node, clock = adapter(monkeypatch)
    node.pose_history.add(9.5, (2., 0., -3.5), 0.)
    sensor.down_callback(scan(9.5, 2.))
    assert sensor.descent_clearance(10.) == pytest.approx(1.)  # 2 - .5 viagem - .5 margem
    clock.value = 10.3
    assert sensor.descent_clearance(clock.value) == 0.
