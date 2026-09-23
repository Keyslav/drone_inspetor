"""Buffer monotônico, snapshots imutáveis e contratos ROS reais do monitor."""

from array import array
from concurrent.futures import ThreadPoolExecutor
import math
from types import SimpleNamespace

import numpy as np
import pytest
from px4_msgs.msg import BatteryStatus, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan

from drone_inspetor_msgs.msg import DroneStateMSG, MissionStateMSG
from drone_inspetor.gui.presentation.telemetry import MONITOR_TOPICS, MonitorStore
from drone_inspetor.subscribers import dashboard_monitor_subscriber as monitor


def test_waiting_then_live_then_stale_without_another_message():
    now = SimpleNamespace(value=100.)
    store = MonitorStore(clock=lambda: now.value)
    initial = store.snapshot()
    assert len(initial) == 9
    assert all(item.age is None and item.health == 'waiting' and item.hz == 0.
               for item in initial.values())
    store.update('local', {'x': 3.})
    now.value += 0.1
    store.update('local', {'x': 4.})
    assert store.snapshot()['local'].hz == pytest.approx(10.)
    now.value += 1.1
    snapshot = store.snapshot()['local']
    assert snapshot.health == 'stale'
    assert snapshot.age == pytest.approx(1.1)
    assert snapshot.values['x'] == 4.
    now.value += 5.
    assert store.snapshot()['local'].hz == 0.


def test_immutable_snapshots_copy_nested_arrays_and_preserve_nan():
    store = MonitorStore()
    values = {'position': np.array([1., 2., math.nan]),
              'nested': {'values': array('f', [3., 4.])}}
    store.update('setpoint', values, now=10.)
    first = store.snapshot(now=10.)['setpoint']
    values['position'][0] = 99.
    values['nested']['values'][0] = 99.
    assert first.values['position'][:2] == (1., 2.)
    assert math.isnan(first.values['position'][2])
    assert first.values['nested']['values'] == (3., 4.)
    with pytest.raises(TypeError):
        first.values['nested']['values'] = ()
    store.update('setpoint', {'position': [9., 8., 7.]}, now=11.)
    assert first.values['position'][0] == 1.


def test_bounded_history_and_duplicate_or_restarted_clock():
    store = MonitorStore(history_size=4, frequency_window=1.)
    for sample in range(10000):
        store.update('local', {'x': sample}, now=sample / 100.)
    assert len(store._arrivals['local']) == 4
    assert store.snapshot(now=99.99)['local'].hz == pytest.approx(100.)
    store.update('local', {}, now=0.)
    store.update('local', {}, now=0.)
    assert store.snapshot(now=0.)['local'].hz == 0.
    with pytest.raises(ValueError):
        store.snapshot(now=math.nan)


def test_updates_and_snapshots_are_atomic_across_threads():
    store = MonitorStore()

    def update(_):
        for index in range(100):
            store.update('local', {'x': index, 'y': index})
            values = store.snapshot()['local'].values
            assert values['x'] == values['y']

    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(update, range(4)))


@pytest.fixture
def adapter(monkeypatch):
    callbacks = {}

    def subscribe(node, spec, callback):
        callbacks[spec.name] = callback
        return SimpleNamespace(topic_name=spec.name, spec=spec)

    monkeypatch.setattr(monitor, 'create_subscription_from', subscribe)
    store = MonitorStore(clock=lambda: 1.)
    subscriber = monitor.DashboardMonitorSubscriber(object(), store)
    return subscriber, store, callbacks


def test_catalog_matches_specs_and_subscriptions_reuse_qos(adapter):
    subscriber, _, callbacks = adapter
    assert len(callbacks) == len(MONITOR_TOPICS) == 9
    for topic in MONITOR_TOPICS:
        spec = monitor.MONITOR_SPECS[topic.key]
        assert topic.topic == spec.name
        assert any(subscription.spec is spec for subscription in subscriber.subscriptions)
        assert spec.name in callbacks


@pytest.mark.parametrize('key,message', [
    ('drone', DroneStateMSG()), ('mission', MissionStateMSG()),
    ('local', VehicleLocalPosition(x=3.)), ('battery', BatteryStatus(remaining=0.5)),
    ('setpoint', TrajectorySetpoint(position=[1., 2., 3.], velocity=[4., 5., 6.])),
])
def test_native_message_fields_survive_conversion(adapter, key, message):
    subscriber, store, _ = adapter
    subscriber._receive(key, message)
    result = store.snapshot()[key].values
    assert set(result) == set(message.get_fields_and_field_types())
    if key == 'setpoint':
        message.position[0] = 99.
        assert result['position'] == (1., 2., 3.)
        assert result['velocity'] == (4., 5., 6.)


def test_status_adds_readable_names_without_replacing_native_ids(adapter):
    subscriber, store, _ = adapter
    message = VehicleStatus(nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                            arming_state=VehicleStatus.ARMING_STATE_ARMED)
    subscriber._receive('status', message)
    values = store.snapshot()['status'].values
    assert values['nav_state'] == VehicleStatus.NAVIGATION_STATE_OFFBOARD
    assert values['nav_state_name'] == 'OFFBOARD'
    assert values['arming_state_name'] == 'ARMED'
    message.nav_state = 255
    subscriber._receive('status', message)
    assert store.snapshot()['status'].values['nav_state_name'] == 'UNKNOWN(255)'


@pytest.mark.parametrize('key', ['lidar', 'down', 'depth'])
def test_scans_include_only_valid_minimum_counts_and_frame(adapter, key):
    subscriber, store, _ = adapter
    scan = LaserScan(range_min=0.1, range_max=10.,
                     ranges=[math.nan, math.inf, -1., 0.05, 20., 2., 3.])
    scan.header.frame_id = 'sensor_link'
    subscriber._receive(key, scan)
    assert dict(store.snapshot()[key].values) == {
        'minimum_distance': 2., 'beam_count': 7, 'valid_count': 2, 'frame_id': 'sensor_link',
    }
    scan.ranges = [math.nan, math.inf]
    subscriber._receive(key, scan)
    values = store.snapshot()[key].values
    assert values['valid_count'] == 0
    assert math.isnan(values['minimum_distance'])
    assert 'ranges' not in values


def test_subscriber_records_resolved_remap_and_keeps_previous_snapshots(monkeypatch):
    store = MonitorStore(clock=lambda: 10.)
    previous = store.snapshot()['local']

    def subscribe(node, spec, callback):
        return SimpleNamespace(topic_name='/drone_28' + spec.name)

    monkeypatch.setattr(monitor, 'create_subscription_from', subscribe)
    monitor.DashboardMonitorSubscriber(object(), store)
    assert store.snapshot()['local'].topic == '/drone_28/fmu/out/vehicle_local_position'
    assert previous.topic == '/fmu/out/vehicle_local_position'
    assert store.snapshot()['local'].health == 'waiting'


def test_republished_source_timestamp_uses_local_reception_age_and_frequency():
    store = MonitorStore()
    store.update('status', {'timestamp': 1}, now=10.)
    store.update('status', {'timestamp': 1}, now=10.5)
    snapshot = store.snapshot(now=10.6)['status']
    assert snapshot.values['timestamp'] == 1
    assert snapshot.age == pytest.approx(0.1)
    assert snapshot.hz == pytest.approx(2.)
    assert snapshot.health == 'live'
