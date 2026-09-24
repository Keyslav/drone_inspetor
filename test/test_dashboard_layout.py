"""Reflow não recria instrumentos nem deixa pixmaps impor largura à janela."""

import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPixmap
from drone_inspetor.gui.dashboard_gui import DashboardGUI
from drone_inspetor.gui.widgets.images import ResponsiveImageLabel
from drone_inspetor.gui.widgets.cockpit import FlightSummary
from drone_inspetor.gui.presentation.telemetry import MonitorStore
from drone_inspetor.signals.dashboard_signals import DashboardSignals


def test_dashboard_reflows_without_recreating_sensors(monkeypatch):
    # Chromium tem sua própria verificação visual; aqui isolamos a geometria Qt.
    class Map(QWidget):
        def __init__(self, parent=None, mapa_manager=None):
            super().__init__(parent)
            self.map_ready = False
    monkeypatch.setattr('drone_inspetor.gui.dashboard_gui.InteractiveMapWidget', Map)
    app = QApplication.instance() or QApplication([])
    gui = DashboardGUI(DashboardSignals())
    try:
        gui.show()
        original = tuple(gui.sensor_panels)
        for width, columns in ((1440, 2), (900, 2), (640, 1), (1440, 2)):
            gui.resize(width, 800)
            app.processEvents()
            assert gui.width() == width
            assert tuple(gui.sensor_panels) == original
            for index, panel in enumerate(original):
                pos = gui.sensor_grid.getItemPosition(gui.sensor_grid.indexOf(panel))
                assert pos[:2] == (index // columns, index % columns)
                assert panel.width() <= gui.scroll.viewport().width()
            assert gui.scroll.horizontalScrollBar().maximum() == 0
    finally:
        gui.close()


def test_image_rescales_without_waiting_for_next_frame():
    app = QApplication.instance() or QApplication([])
    label = ResponsiveImageLabel()
    label.show()
    pixmap = QPixmap(1280, 720)
    label.set_source_pixmap(pixmap)
    for width, height in ((640, 360), (300, 400), (960, 540)):
        label.resize(width, height)
        app.processEvents()
        rendered = label.pixmap()
        assert rendered.width() <= width and rendered.height() <= height
        assert abs(rendered.width() / rendered.height() - 16 / 9) < .02
    label.close()


def test_summary_expires_telemetry_instead_of_freezing_live_values():
    app = QApplication.instance() or QApplication([])
    now = [0.]
    store = MonitorStore(clock=lambda: now[0])
    store.update('local', {'vx': 3., 'vy': 0., 'vz': 0.})
    summary = FlightSummary(store)
    assert summary.values[2].text() == '3.0 m/s'
    now[0] = 5.
    summary.refresh()
    assert summary.values[2].text() == '—'
    assert summary.notes[2].text() == 'Dados desatualizados'
    summary.close()
