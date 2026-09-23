"""Propriedade e ciclo de vida de mídia sem codecs ou dispositivos físicos."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event

import numpy as np
import pytest

from drone_inspetor.media.photos import filename_component, save_photo
from drone_inspetor.media.recorder import VideoRecorder


class FakeWriter:
    def __init__(self, opened=True):
        self.opened = opened
        self.released = 0
        self.frames = []
        self.entered = Event()
        self.finish = Event()
        self.block = False

    def isOpened(self):
        return self.opened

    def write(self, frame):
        assert not self.released
        self.entered.set()
        if self.block:
            assert self.finish.wait(2)
        assert not self.released
        self.frames.append(frame.copy())

    def release(self):
        self.released += 1


def test_close_waits_for_active_write_and_no_write_after_close(tmp_path):
    writer = FakeWriter()
    writer.block = True
    recorder = VideoRecorder('MJPG', 15, writer_factory=lambda *args: writer)
    recorder.start(tmp_path / 'mission.avi')
    frame = np.zeros((8, 10, 3), dtype=np.uint8)
    with ThreadPoolExecutor(2) as executor:
        writing = executor.submit(recorder.write, frame)
        assert writer.entered.wait(1)
        close_started = Event()

        def close():
            close_started.set()
            return recorder.close()

        closing = executor.submit(close)
        assert close_started.wait(1)
        assert not closing.done()
        assert writer.released == 0
        writer.finish.set()
        assert writing.result(timeout=1)
        assert closing.result(timeout=1).endswith('mission.avi')
    assert writer.released == 1
    assert recorder.write(frame) is False
    recorder.close()
    assert writer.released == 1


def test_failed_open_releases_writer_and_allows_retry(tmp_path):
    failed = FakeWriter(opened=False)
    ready = FakeWriter()
    writers = iter((failed, ready))
    recorder = VideoRecorder('mp4v', 30, writer_factory=lambda *args: next(writers))
    with pytest.raises(OSError):
        recorder.start(tmp_path / 'failed.mp4', frame_size=(10, 8))
    assert not recorder.active
    assert failed.released == 1
    recorder.start(tmp_path / 'ok.mp4', frame_size=(10, 8))
    assert recorder.opened
    recorder.close()
    assert ready.released == 1


def test_second_start_does_not_abandon_existing_session(tmp_path):
    writer = FakeWriter()
    recorder = VideoRecorder('MJPG', 15, writer_factory=lambda *args: writer)
    recorder.start(tmp_path / 'one.avi', frame_size=(10, 8))
    with pytest.raises(RuntimeError):
        recorder.start(tmp_path / 'two.avi')
    assert recorder.close().endswith('one.avi')
    assert writer.released == 1


def test_resolution_change_is_resized_and_lazy_close_does_not_open(tmp_path):
    writer = FakeWriter()
    recorder = VideoRecorder('MJPG', 15, writer_factory=lambda *args: writer)
    recorder.start(tmp_path / 'one.avi')
    assert recorder.active and not recorder.opened
    recorder.write(np.zeros((8, 10, 3), dtype=np.uint8))
    recorder.write(np.zeros((16, 20, 3), dtype=np.uint8))
    assert [frame.shape for frame in writer.frames] == [(8, 10, 3), (8, 10, 3)]
    recorder.close()
    recorder.start(tmp_path / 'empty.avi')
    recorder.close()
    assert writer.released == 1


def test_write_failure_closes_session(tmp_path):
    class FailedWriter(FakeWriter):
        def write(self, frame):
            raise OSError('Disk full')

    writer = FailedWriter()
    recorder = VideoRecorder('MJPG', 15, writer_factory=lambda *args: writer)
    recorder.start(tmp_path / 'failed.avi')
    with pytest.raises(OSError):
        recorder.write(np.zeros((8, 10, 3), dtype=np.uint8))
    assert not recorder.active and writer.released == 1


def test_photo_failure_is_reported_and_names_cannot_escape_folder(monkeypatch, tmp_path):
    monkeypatch.setattr('drone_inspetor.media.photos.cv2.imwrite', lambda *args: False)
    with pytest.raises(OSError):
        save_photo(tmp_path / 'photo.jpg', np.zeros((8, 10, 3), dtype=np.uint8))
    assert '/' not in filename_component('../../flare/corrosao')


@pytest.mark.parametrize('codec,fps', [('x', 20), ('MJPG', 0), ('MJPG', float('nan'))])
def test_invalid_video_configuration(codec, fps):
    with pytest.raises(ValueError):
        VideoRecorder(codec, fps)
