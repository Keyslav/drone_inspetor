"""Contratos dos sensores com mensagens reais, sem iniciar nós ou hardware."""

import math
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('drone_inspetor_msgs.msg')

from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan

from drone_inspetor.nodes.depth_node.depth_node import DepthNode
from drone_inspetor.nodes.depth_node.projection import DepthCalibration
from drone_inspetor.nodes.lidar_node.lidar_node import LidarData, LidarNode
from drone_inspetor.nodes.lidar_node.processing import sector_flags


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def depth_node(calibrated=True):
    node = SimpleNamespace(
        bridge=CvBridge(), _scan_enabled=True, _scan_frame_id='depth_scan_flu',
        _calibration=DepthCalibration(horizontal_fov_deg=90. if calibrated else 0., bins=3),
        min_depth=.1, max_depth=10., visualization_mode='grayscale', proximity_threshold=1.,
        _obstacle_flags=sector_flags([], []),
        get_logger=lambda: SimpleNamespace(error=lambda _: None),
    )
    for name in ('scan_publisher', 'processed_depth_publisher', 'depth_stats_publisher',
                 'proximity_alerts_publisher', 'obstacles_publisher'):
        setattr(node, name, Publisher())
    for name in ('process_depth_image', 'publish_obstacles', 'publish_depth_statistics',
                 'publish_proximity_alerts'):
        setattr(node, name, MethodType(getattr(DepthNode, name), node))
    return node


def test_metric_scan_preserves_acquisition_stamp_and_explicit_flu_frame():
    node = depth_node()
    message = node.bridge.cv2_to_imgmsg(np.full((3, 3), 2., dtype=np.float32), encoding='32FC1')
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.header.frame_id = 'depth_optical_frame'
    DepthNode.depth_image_callback(node, message)
    scan = node.scan_publisher.messages[0]
    assert scan.header.stamp == message.header.stamp
    assert scan.header.frame_id == 'depth_scan_flu'
    assert scan.angle_min < 0 < scan.angle_max and scan.angle_increment > 0
    assert scan.ranges[1] == 2.
    assert node.processed_depth_publisher.messages[0].header == message.header
    assert len(node.scan_publisher.messages) == 1


def test_uncalibrated_sensor_keeps_dashboard_without_navigation_scan():
    node = depth_node(calibrated=False)
    message = node.bridge.cv2_to_imgmsg(np.full((3, 3), 2000, dtype=np.uint16), encoding='16UC1')
    DepthNode.depth_image_callback(node, message)
    assert node.scan_publisher.messages == []
    assert len(node.processed_depth_publisher.messages) == 1
    assert node.depth_statistics['mean_distance'] == 2.


def test_empty_alert_list_is_published_to_clear_previous_alert():
    node = depth_node()
    for depth in (.5, 5.):
        message = node.bridge.cv2_to_imgmsg(np.full((3, 3), depth, dtype=np.float32), encoding='32FC1')
        DepthNode.depth_image_callback(node, message)
    assert node.proximity_alerts_publisher.messages[0].data != '[]'
    assert node.proximity_alerts_publisher.messages[1].data == '[]'
    assert not node.obstacles_publisher.messages[-1].have_obstacles_1m


def test_visualization_failure_cannot_drop_metric_scan():
    node = depth_node()

    def fail(*args):
        raise RuntimeError('Renderer failed')

    node.process_depth_image = fail
    message = node.bridge.cv2_to_imgmsg(np.full((3, 3), 2., dtype=np.float32), encoding='32FC1')
    DepthNode.depth_image_callback(node, message)
    assert len(node.scan_publisher.messages) == 1
    assert len(node.processed_depth_publisher.messages) == 0


def lidar_node(clock):
    node = SimpleNamespace(
        _timeout=.75, lidar_data=LidarData(), _horizontal_flags=sector_flags([], []),
        _received={'horizontal': None, 'down': None}, _stamps={}, _generation=0,
        _data_published=-1, _flags_published=-1,
        lidar_data_publisher=Publisher(), obstacles_publisher=Publisher(),
        get_logger=lambda: SimpleNamespace(error=lambda _: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0] * 1e9))),
    )
    for name in ('_accept', '_expire', '_to_obstacles_msg'):
        setattr(node, name, MethodType(getattr(LidarNode, name), node))
    return node


def test_lidar_does_not_renew_expired_or_duplicate_observations(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('drone_inspetor.nodes.lidar_node.lidar_node.time.monotonic', lambda: clock[0])
    node = lidar_node(clock)
    message = LaserScan()
    message.header.stamp.sec = 100
    message.range_min, message.range_max = .1, 10.
    message.angle_min, message.angle_increment = 0., .1
    message.ranges = [.5]
    LidarNode.laserscan_callback(node, message)
    LidarNode.publish_lidar_data(node)
    LidarNode.publish_obstacles(node)
    assert node.obstacles_publisher.messages[-1].have_obstacles_front_90
    assert len(node.lidar_data_publisher.messages) == 1
    clock[0] += .5
    LidarNode.laserscan_callback(node, message)
    LidarNode.publish_lidar_data(node)
    LidarNode.publish_obstacles(node)
    assert len(node.lidar_data_publisher.messages) == 1
    assert node._received['horizontal'] == 100.
    clock[0] += .5
    LidarNode.publish_lidar_data(node)
    LidarNode.publish_obstacles(node)
    assert list(node.lidar_data_publisher.messages[-1].point_vector) == []
    assert not node.obstacles_publisher.messages[-1].have_obstacles_front_90
    assert len(node.lidar_data_publisher.messages) == 2
    LidarNode.publish_lidar_data(node)
    assert len(node.lidar_data_publisher.messages) == 2


def test_invalid_down_scan_replaces_previous_ground_distance():
    clock = [100.]
    node = lidar_node(clock)
    message = LaserScan()
    message.range_min, message.range_max = .1, 10.
    message.ranges = [.3]
    LidarNode.lidar_down_callback(node, message)
    assert node.lidar_data.ground_distance == pytest.approx(.3)
    message.ranges = [math.nan]
    LidarNode.lidar_down_callback(node, message)
    assert math.isnan(node.lidar_data.ground_distance)
    assert not node._to_obstacles_msg().have_obstacles_down_1m


def test_lidar_accepts_new_epoch_after_simulation_clock_resets(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('drone_inspetor.nodes.lidar_node.lidar_node.time.monotonic', lambda: 500.)
    node = lidar_node(clock)
    message = LaserScan()
    message.range_min, message.range_max = .1, 10.
    message.angle_min, message.angle_increment = 0., .1
    message.header.stamp.sec = 100
    message.ranges = [.5]
    LidarNode.laserscan_callback(node, message)
    clock[0] = 2.
    message.header.stamp.sec = 2
    message.ranges = [3.]
    LidarNode.laserscan_callback(node, message)
    assert node._stamps['horizontal'] == 2.
    assert node.lidar_data.point_vector == [3., 0.]
    assert not node._horizontal_flags['have_obstacles_1m']
