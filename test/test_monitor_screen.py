"""Valida ausência/expiração, unidades e ciclo de vida do monitor Qt."""

import os

from PyQt6.QtWidgets import QApplication
import pytest

from drone_inspetor.gui.monitor_screen import MonitorWindow
from drone_inspetor.gui.presentation.telemetry import MonitorStore


@pytest.fixture(scope='module')
def qt_app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    yield QApplication.instance() or QApplication([])


@pytest.fixture
def monitor(qt_app):
    now = [10.]
    store = MonitorStore(clock=lambda: now[0])
    window = MonitorWindow(store, clock=lambda: now[0])
    yield window, store, now
    window.close()
    qt_app.processEvents()


def feed(store):
    store.update('drone', {'state_name': 'EM_VOO', 'is_landed': False,
                           'current_yaw_deg': 92., 'is_on_trajectory': True})
    store.update('status', {'nav_state_name': 'OFFBOARD', 'arming_state_name': 'ARMED',
                            'failsafe': False})
    store.update('local', {'x': 12., 'y': 3., 'z': -8., 'vx': 3., 'vy': 4., 'vz': -2.,
                           'xy_valid': True, 'z_valid': True,
                           'v_xy_valid': True, 'v_z_valid': True})
    store.update('battery', {'connected': True, 'remaining': .74,
                             'voltage_v': 15.8, 'current_a': 6., 'warning': 0})
    store.update('mission', {'mission_name': 'Inspeção de estruturas',
                             'state_name': 'EXECUTANDO_INSPECIONANDO', 'on_mission': True,
                             'ponto_de_inspecao_indice_atual': 1,
                             'total_pontos_de_inspecao': 5, 'objeto_alvo': 'Flare'})
    store.update('setpoint', {'position': [15., 3., -8.], 'velocity': [2., 0., 0.],
                              'acceleration': [.5, 0., 0.]})


def test_missing_and_expired_data_never_look_like_current_zero(monitor):
    window, store, now = monitor
    assert window.cards['speed'][0].text() == '—'
    assert window.fields['failsafe'].text() == '—'
    feed(store)
    window.refresh()
    assert window.cards['battery'][0].text() == '74 %'
    now[0] += 5
    window.refresh()
    assert window.cards['battery'][0].text() == '—'
    assert 'Desatualizado' in window.cards['battery'][1].text()
    assert window.fields['failsafe'].text() == '—'
    assert window.history.samples[-1][1:] == (None, None)
    # Última amostra continua disponível na inspeção, marcada como antiga.
    assert 'Desatualizado' in window.detail_status.text()
    assert 'posições locais Norte/Leste/Cima' in window.coordinate_note.text()
    assert window.tree.topLevelItemCount() > 0


def test_ned_units_validity_and_reference_are_distinct(monitor):
    window, store, _ = monitor
    feed(store)
    window.refresh()
    assert window.fields['height'].text() == '8.00 m'
    assert window.fields['climb'].text() == '2.00 m/s'
    assert window.fields['position'].text() == '12.00 / 3.00 / -8.00 m'
    assert window.fields['reference'].text() == '15.00 / 3.00 / -8.00 m'
    assert window.fields['waypoint'].text() == '2 de 5'
    store.update('local', {'z': -8., 'z_valid': False, 'vx': 0., 'v_xy_valid': False})
    store.update('battery', {'connected': True, 'remaining': -1})
    window.refresh()
    assert window.fields['height'].text() == '—'
    assert window.cards['speed'][0].text() == '—'
    assert window.cards['battery'][0].text() == '—'


def test_failsafe_filter_and_reopen(monitor, qt_app):
    window, store, _ = monitor
    feed(store)
    store.update('status', {'failsafe': True, 'nav_state_name': 'AUTO_RTL'})
    window.show()
    qt_app.processEvents()
    assert 'Failsafe ativo' in window.banner.text()
    row = window.keys.index('status')
    window.topic_table.selectRow(row)
    window.search.setText('failsafe')
    assert window.tree.topLevelItemCount() == 1
    assert window.tree.topLevelItem(0).text(1) == 'True'
    window.close()
    assert not window.timer.isActive()
    window.show()
    qt_app.processEvents()
    assert window.timer.isActive()
    assert window.topic_table.currentRow() == row
    assert window.search.text() == 'failsafe'


def test_history_is_bounded_and_breaks_for_nan_reference(monitor):
    window, store, now = monitor
    feed(store)
    store.update('setpoint', {'velocity': [float('nan'), 0., 0.]})
    window.refresh()
    assert window.history.samples[-1][2] is None
    for _ in range(400):
        now[0] += .2
        window.history.append(now[0], 1., 2.)
    assert len(window.history.samples) <= 301
    assert window.history.samples[-1][0] - window.history.samples[0][0] <= 60
