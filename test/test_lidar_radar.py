"""Contratos visuais do radar: FLU, resize, dados inválidos e expiração."""

import math
import os

import pytest
from PyQt6.QtCore import QPointF
from PyQt6.QtWidgets import QApplication, QLabel

from drone_inspetor.gui.lidar_screen import LidarScreen
from drone_inspetor.gui.widgets.lidar_radar import LidarRadar, polar_to_screen, valid_points


@pytest.fixture(scope='module')
def qt_app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    app = QApplication.instance() or QApplication([])
    yield app
    app.processEvents()


def test_lidar_points_keep_meters_radians_and_reject_invalid_returns():
    vector = [2., math.pi / 2, math.inf, 0., 0., 0., 3., math.nan, 4.]
    assert valid_points(vector) == [(2., math.pi / 2)]
    assert valid_points(None) == []


@pytest.mark.parametrize('angle, expected', [
    (0., (100., 50.)), (math.pi / 2, (50., 100.)),
    (-math.pi / 2, (150., 100.)), (math.pi, (100., 150.)),
])
def test_flu_directions_are_not_mirrored(angle, expected):
    point = polar_to_screen(5., angle, QPointF(100., 100.), 10.)
    assert (point.x(), point.y()) == pytest.approx(expected)


@pytest.mark.parametrize('size', [(220, 240), (600, 240), (280, 600)])
def test_radar_circle_fits_in_wide_and_tall_panels(qt_app, size):
    radar = LidarRadar()
    radar.resize(*size)
    radar.set_points([2., 0., 4., math.pi / 2])
    radar.show()
    qt_app.processEvents()
    center, radius = radar.canvas.radar_geometry()
    assert radius > 0
    assert center.x() - radius >= 20
    assert center.y() - radius >= 20
    assert center.x() + radius <= radar.canvas.width() - 20
    assert center.y() + radius <= radar.canvas.height() - 20
    assert radar.size().width() == size[0]
    assert not radar.grab().isNull()  # Executa paintEvent, inclusive em Qt offscreen.
    radar.close()


def test_empty_scan_and_stale_reception_cannot_look_like_clear_space(qt_app):
    now = [10.]
    radar = LidarRadar(clock=lambda: now[0])
    assert radar.status_label.text() == 'Aguardando LiDAR'
    radar.set_points([2., 0.])
    radar.set_ground_distance(1.2)
    assert radar.has_fresh_points()
    assert radar.nearest_label.text() == 'Mín.: 2.00 m'
    now[0] += 2.
    radar.refresh_status()
    assert not radar.has_fresh_points()
    assert radar.status_label.text() == 'Recepção atrasada'
    assert radar.nearest_label.text() == 'Mín.: —'
    assert radar.ground_label.text() == 'Abaixo: —'
    radar.set_points([])
    assert radar.status_label.text() == 'Sem retornos válidos'
    radar.close()


def test_screen_public_callbacks_and_range_selector(qt_app):
    label = QLabel()
    screen = LidarScreen(None, label)
    screen.update_point_vector([3., 0.])
    screen.update_ground_distance(2.5)
    screen.update_lidar_statistics({})
    screen.update_obstacle_detections({})
    assert screen.radar.points == [(3., 0.)]
    assert screen.radar.ground_label.text() == 'Abaixo: 2.50 m'
    screen.radar.range_selector.setCurrentIndex(0)
    assert screen.radar.display_range == 3.
    for invalid in (None, math.nan, math.inf, -1.):
        screen.update_ground_distance(invalid)
        assert screen.radar.ground_label.text() == 'Abaixo: —'
    label.close()
    screen.close()
