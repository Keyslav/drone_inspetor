"""Percepção testada com inferência falsa, sem ROS, pesos ou GPU."""

from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import json
from pathlib import Path
from threading import Event
import time
from types import SimpleNamespace

import numpy as np
import pytest

from drone_inspetor.nodes.cv_node.detection_buffer import DetectionBuffer
from drone_inspetor.nodes.cv_node.inference import DetectionPipeline, InferenceOptions
from drone_inspetor.nodes.cv_node.model_registry import ModelManager, ModelRegistry


DETECTION = {'class': 'Flare', 'object_type': 'flare', 'confidence': 0.9,
             'bbox': [0, 0, 10, 10], 'bbox_center': [5., 5.]}


def waiting(buffer, timeout=0.2):
    """Inicia a consulta garantindo que ela entrou na espera antes do teste seguir."""
    started = Event()
    original_wait = buffer._condition.wait

    def wait(*args):
        started.set()
        return original_wait(*args)

    buffer._condition.wait = wait
    executor = ThreadPoolExecutor(1)
    future = executor.submit(buffer.wait_for, 'Flare', timeout)
    assert started.wait(1)
    return executor, future


def test_detection_ignores_cached_frame_and_waits_for_new_observation():
    buffer = DetectionBuffer()
    buffer.publish([DETECTION], time.monotonic())
    executor, future = waiting(buffer)
    with executor:
        assert not future.done()
        buffer.publish([DETECTION], time.monotonic())
        result = future.result(timeout=1)
    assert result['confidence'] == 0.9
    result['bbox'][0] = 999
    assert buffer._detections[0]['bbox'][0] == 0


def test_detection_timeout_is_monotonic_and_slow_inference_is_stale():
    buffer = DetectionBuffer(max_frame_age=0.005)
    started = time.monotonic()
    executor, future = waiting(buffer, 0.07)
    with executor:
        received = time.monotonic()
        time.sleep(0.01)
        buffer.publish([DETECTION], received)
        assert future.result(timeout=1) is None
    assert 0.05 <= time.monotonic() - started < 1.0


def test_shutdown_interrupts_long_query_without_waiting_for_deadline():
    buffer = DetectionBuffer()
    executor, future = waiting(buffer, 600)
    with executor:
        buffer.close()
        assert future.result(timeout=0.5) is None
    buffer.publish([DETECTION], time.monotonic())
    assert buffer.wait_for('flare', 1) is None


@pytest.mark.parametrize('timeout', [0, float('inf'), float('nan'), -1])
def test_invalid_timeout_rejected(timeout):
    with pytest.raises(ValueError):
        DetectionBuffer().wait_for('flare', timeout)


def registry(tmp_path):
    entries = {'equipment': [{'file_name': 'object.pt'}, {'file_name': 'object2.pt'}],
               'anomaly': [{'file_name': 'anomaly.pt'}, {'file_name': 'failed.pt'}]}
    (tmp_path / 'models.json').write_text(json.dumps({'models': entries}), encoding='utf-8')
    for name in ('object.pt', 'object2.pt', 'anomaly.pt', 'failed.pt'):
        (tmp_path / name).touch()
    return ModelRegistry(tmp_path)


def test_registry_validates_category_and_prevents_arbitrary_path(tmp_path):
    catalog = registry(tmp_path)
    assert catalog.first('equipment') == 'object.pt'
    assert len(catalog.entries) == 4
    for filename, kind in (('../object.pt', 'equipment'), ('object.pt', 'anomaly')):
        with pytest.raises(ValueError):
            catalog.path(filename, kind)
    catalog.entries.clear()
    assert len(catalog.entries) == 4


def test_registry_accepts_colcon_symlink_install(tmp_path):
    build = tmp_path / 'build'
    build.mkdir()
    registry(build)
    share = tmp_path / 'share'
    share.mkdir()
    for resource in build.iterdir():
        (share / resource.name).symlink_to(resource)
    installed = ModelRegistry(share)
    assert installed.path('object.pt', 'equipment').samefile(build / 'object.pt')
    (build / 'object.pt').unlink()
    with pytest.raises(FileNotFoundError):
        installed.path('object.pt', 'equipment')


def test_failed_pair_replacement_preserves_both_models(tmp_path):
    def loader(path):
        if Path(path).name == 'failed.pt':
            raise OSError('Invalid model')
        return object()

    models = ModelManager(registry(tmp_path), loader)
    models.replace('object.pt', 'anomaly.pt')
    with models.snapshot() as original:
        pass
    with pytest.raises(OSError):
        models.replace('object2.pt', 'failed.pt')
    with models.snapshot() as current:
        assert current == original
    assert models.filenames == ('object.pt', 'anomaly.pt')


def test_model_replacement_cannot_mix_weights_and_names_mid_frame(tmp_path):
    models = ModelManager(registry(tmp_path), lambda path: path)
    models.replace('object.pt', 'anomaly.pt')
    with ThreadPoolExecutor(1) as executor:
        with models.snapshot() as original:
            started = Event()

            def replace():
                started.set()
                models.replace('', 'failed.pt')

            changed = executor.submit(replace)
            assert started.wait(1)
            assert not changed.done()
            assert original[1].endswith('anomaly.pt')
        changed.result(timeout=1)
    assert models.filenames == ('object.pt', 'failed.pt')


class Tensor:
    def __init__(self, values):
        self.values = np.asarray(values)

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class Boxes:
    def __init__(self, coords, confidence=None):
        self.xyxy = Tensor(coords)
        self.conf = Tensor(confidence if confidence is not None else [0.9] * len(coords))
        self.cls = Tensor([0] * len(coords))
        self.size = len(coords)

    def __len__(self):
        return self.size


class Predictor:
    def __init__(self, name, boxes):
        self.names = {0: name}
        self.result = SimpleNamespace(boxes=boxes, masks=None)
        self.calls = []

    def predict(self, frame, **kwargs):
        self.calls.append((frame.copy(), kwargs))
        return [self.result]


class Models:
    def __init__(self, *models):
        self.models = models

    @contextmanager
    def snapshot(self):
        yield self.models


def test_pipeline_clamps_boxes_and_preserves_cpu_selection_and_anomaly_offsets():
    objects = Predictor('Flare', Boxes([[-3, 2, 8, 15], [30, 2, 40, 4]]))
    anomalies = Predictor('Rust', Boxes([[1, 2, 50, 80]]))
    pipeline = DetectionPipeline(Models(objects, anomalies))
    image = np.zeros((12, 20, 3), dtype=np.uint8)
    annotated, detections, captures = pipeline.process(
        image, InferenceOptions(enable_anomalies=True, device='cpu'),
    )
    assert len(detections) == len(captures) == 1
    assert objects.calls[0][1]['device'] == anomalies.calls[0][1]['device'] == 'cpu'
    assert detections[0]['bbox'] == [0, 2, 8, 12]
    assert detections[0]['anomalies'][0]['bbox'] == [1, 4, 8, 12]
    assert anomalies.calls[0][0].shape == (10, 8, 3)
    assert np.count_nonzero(image) == 0
    assert np.count_nonzero(annotated) > 0
    assert captures[0].crop.shape == (10, 8, 3)


def test_unknown_target_does_not_infer_another_class():
    objects = Predictor('Flare', Boxes([[0, 0, 10, 10]]))
    pipeline = DetectionPipeline(Models(objects, None))
    image = np.zeros((12, 20, 3), dtype=np.uint8)
    _, detections, captures = pipeline.process(
        image, InferenceOptions(target='Unknown', filter_target=True),
    )
    assert detections == captures == []
    assert objects.calls == []


def test_no_anomaly_inference_until_enabled():
    objects = Predictor('Flare', Boxes([[0, 0, 10, 10]]))
    anomalies = Predictor('Rust', Boxes([[1, 1, 5, 5]]))
    pipeline = DetectionPipeline(Models(objects, anomalies))
    _, detections, captures = pipeline.process(
        np.zeros((12, 20, 3), dtype=np.uint8), InferenceOptions(),
    )
    assert detections[0]['anomalies'] == []
    assert captures == [] and anomalies.calls == []


def test_mission_or_model_change_invalidates_a_pending_detection():
    buffer = DetectionBuffer()
    executor, future = waiting(buffer, 60)
    with executor:
        buffer.invalidate()
        assert future.result(timeout=0.5) is None
