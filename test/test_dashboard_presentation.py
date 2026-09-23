"""Contratos da apresentação e memória de frames sem iniciar hardware ROS."""

from dataclasses import FrozenInstanceError
import os
from types import SimpleNamespace

import numpy as np
import pytest

from PyQt6.QtWidgets import QApplication, QLabel
from drone_inspetor.gui.cv_screen import CVScreen
from drone_inspetor.gui.presentation.detections import DetectionFrame, format_analysis_report
from drone_inspetor.gui.widgets.images import ImageProcessor
from drone_inspetor.signals.dashboard_signals import CVSignals
from drone_inspetor.subscribers.dashboard_cv_subscriber import DashboardCVSubscriber


@pytest.fixture(scope='module')
def qt_app():
    """Qt usa apenas o plugin offscreen, sem servidor gráfico."""
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    app = QApplication.instance() or QApplication([])
    yield app
    app.processEvents()


def detection_message():
    """Cria o contrato ROS real para evitar divergências de campos/tipos."""
    from drone_inspetor_msgs.msg import CVDetectionMSG, CVDetectionItemMSG

    item = CVDetectionItemMSG(
        object_type='equipment', class_name='flare', confidence=0.9,
        bbox=[1, 2, 3, 4], bbox_center=[2, 3],
    )
    return CVDetectionMSG(timestamp='2026-09-20T12:00:00', count=1, detections=[item])


def test_callback_emits_immutable_snapshot(qt_app):
    signals = CVSignals()
    received = []
    signals.detections_received.connect(received.append)
    subscriber = SimpleNamespace(signals=signals)
    message = detection_message()
    DashboardCVSubscriber.cv_detections_callback(subscriber, message)
    frame = received[0]
    message.detections[0].bbox[0] = 99
    assert frame.timestamp == '2026-09-20T12:00:00'
    assert frame.count == 1
    assert frame.detections[0].bbox == (1., 2., 3., 4.)
    with pytest.raises(FrozenInstanceError):
        frame.timestamp = 'outro'


def test_report_counts_detections_instead_of_dictionary_keys():
    frame = DetectionFrame.from_message(detection_message())
    text = format_analysis_report(frame, [], '12:00:00')
    assert 'Detecções atuais: 1' in text
    assert '1. flare (confiança: 0.90)' in text
    assert 'Total de análises: 0' in text


@pytest.mark.parametrize('channels', [1, 3, 4])
def test_qimage_owns_pixels_after_array_reuse(channels):
    shape = (4, 8) if channels == 1 else (4, 8, channels)
    frame = np.full(shape, 128, dtype=np.uint8)
    if channels == 4:
        frame[:, :, 3] = 255
    image = ImageProcessor().cv_to_qimage(frame[:, ::2])
    frame[:] = 0
    assert (image.width(), image.height()) == (4, 4)
    assert image.pixelColor(0, 0).red() == 128


def test_cv_window_open_updates_reset_and_close(qt_app):
    screen = CVScreen(CVSignals(), QLabel())
    screen.update_detections(DetectionFrame.from_message(detection_message()))
    screen.show_analysis_logs()
    assert screen.analysis_window.isVisible()
    assert '1. flare' in screen.analysis_text.toPlainText()
    screen.clear_analysis_logs()
    assert 'Detecções atuais: 0' in screen.analysis_text.toPlainText()
    screen.close()
    assert not screen.analysis_window.isVisible()
    screen.show_analysis_logs()
    assert screen.analysis_window.isVisible()
    screen.close()


def test_model_selection_persists_when_expanded_window_reopens(qt_app):
    signals = CVSignals()
    screen = CVScreen(signals, QLabel())
    screen.model_selector.update_models({
        'models_data_json': '[{"file_name":"equip.pt","object_type":"equipment",'
                            '"name":"Equipamento"}, {"file_name":"anom.pt",'
                            '"object_type":"anomaly","name":"Anomalia"}]',
        'current_object_model': 'equip.pt', 'current_anomaly_model': 'anom.pt',
    })
    screen.expand_screen()
    assert screen.model_selector._selected_equipment_model == 'equip.pt'
    assert screen.model_selector._selected_anomaly_model == 'anom.pt'
    assert screen.model_selector.equip_details_group.field_labels['name'].text() == 'Equipamento'
    screen.close()
    screen.expand_screen()
    assert screen.model_selector._equipment_dropdown.currentData() == 'equip.pt'
    assert screen.model_selector._anomaly_dropdown.currentData() == 'anom.pt'
    screen.close()
