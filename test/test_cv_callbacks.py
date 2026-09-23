"""Contratos ROS reais exercitados sem executor, middleware ou nó físico."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event
from types import SimpleNamespace
import time

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('drone_inspetor_msgs.srv')

from drone_inspetor.media.recorder import VideoRecorder
from drone_inspetor.nodes.camera_node.camera_node import CameraNode
from drone_inspetor.nodes.cv_node.cv_node import CVNode
from drone_inspetor.nodes.cv_node.detection_buffer import DetectionBuffer
from drone_inspetor_msgs.msg import MissionStateMSG
from drone_inspetor_msgs.srv import CVDetectionSRV, RecordDetectionsSRV


class Writer:
    def __init__(self):
        self.releases = 0
        self.writes = 0

    def isOpened(self):
        return True

    def write(self, frame):
        assert not self.releases
        self.writes += 1

    def release(self):
        self.releases += 1


def test_detection_service_fills_generated_response_from_new_frame():
    buffer = DetectionBuffer()
    waiting = Event()
    original_wait = buffer._condition.wait

    def wait(*args):
        waiting.set()
        return original_wait(*args)

    buffer._condition.wait = wait
    node = SimpleNamespace(_detections=buffer, _is_shutting_down=False)
    request = CVDetectionSRV.Request()
    request.object_name = 'Flare'
    request.timeout_seconds = 1.0
    with ThreadPoolExecutor(1) as executor:
        response = executor.submit(CVNode.detection_service_callback, node,
                                   request, CVDetectionSRV.Response())
        assert waiting.wait(1)
        buffer.publish([{'class': 'Flare', 'confidence': 0.85,
                         'bbox': [1, 2, 11, 12], 'bbox_center': [6., 7.]}], time.monotonic())
        result = response.result(timeout=1)
    assert result.success and result.confidence == pytest.approx(0.85)
    assert list(result.bbox) == [1, 2, 11, 12]
    assert list(result.bbox_center) == [6., 7.]


def test_record_services_use_generated_response_and_release_once(tmp_path):
    writer = Writer()
    recorder = VideoRecorder('MJPG', 15, writer_factory=lambda *args: writer)
    node = SimpleNamespace(
        _recorder=recorder, _videos_folder=tmp_path, _is_shutting_down=False,
        get_logger=lambda: SimpleNamespace(error=lambda _: None),
    )
    request = RecordDetectionsSRV.Request()
    request.start_recording = True
    response = CVNode.record_service_callback(node, request, RecordDetectionsSRV.Response())
    assert response.success and response.video_path.startswith(str(tmp_path))
    request.start_recording = False
    stopped = CVNode.record_service_callback(node, request, RecordDetectionsSRV.Response())
    assert stopped.success and stopped.video_path == response.video_path
    recorder.close()
    assert writer.releases == 1


def test_camera_ends_recording_when_mission_ends_even_without_ready_state(tmp_path):
    writer = Writer()
    recorder = VideoRecorder('MJPG', 15, writer_factory=lambda *args: writer)
    recorder.start(tmp_path / 'mission.avi')
    recorder.write(np.zeros((8, 8, 3), dtype=np.uint8))
    statuses = []
    node = SimpleNamespace(
        _recorder=recorder, _recording_status=True, _is_shutting_down=False,
        _on_mission=True, _current_mission_state='EXECUTANDO_RETORNANDO',
        _mission_folder_path=str(tmp_path), _photos_folder=tmp_path, _videos_folder=tmp_path,
        _publish_recording_status=statuses.append,
    )
    node._stop_video_recording = lambda: CameraNode._stop_video_recording(node)
    message = MissionStateMSG()
    message.on_mission = False
    message.state_name = 'ERRO'
    CameraNode.mission_state_callback(node, message)
    assert not recorder.active and writer.releases == 1
    assert statuses == [False]
    assert node._videos_folder is None


def test_cpu_autoselection_does_not_require_cuda(monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(
        cuda=SimpleNamespace(is_available=lambda: False)))
    assert CVNode._select_device('auto') == 'cpu'
    assert CVNode._select_device('cpu') == 'cpu'


def test_repeated_source_frame_does_not_renew_age_with_paused_ros_clock():
    from sensor_msgs.msg import CompressedImage
    node = SimpleNamespace(
        _last_source_stamp=None, _max_frame_age=1., _detections=DetectionBuffer(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
    )
    message = CompressedImage()
    message.header.stamp.sec = 100
    assert CVNode._frame_acquisition_time(node, message) is not None
    assert CVNode._frame_acquisition_time(node, message) is None
    message.header.stamp.sec = 98
    assert CVNode._frame_acquisition_time(node, message) is None
