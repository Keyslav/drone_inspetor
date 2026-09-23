"""Contrato com a página: ordem, serialização e callbacks sem carregar Chromium."""

import math

import pytest

from drone_inspetor.gui.presentation.map_javascript import MapJavaScript


def test_mission_commands_wait_for_page_and_keep_order():
    scripts = []
    page = type('Page', (), {'runJavaScript': lambda self, *args: scripts.append(args)})()
    ready = [False]
    bridge = MapJavaScript(lambda: page, lambda: ready[0])
    bridge.call('clearMissionMarkers', queue=True)
    bridge.call('addInspectionPoint', 1, -22.6, -40.1, True, queue=True)
    bridge.flush()
    assert scripts == []
    ready[0] = True
    bridge.flush()
    bridge.flush()
    assert scripts == [('clearMissionMarkers();',),
                       ('addInspectionPoint(1, -22.6, -40.1, true);',)]


def test_callback_and_status_reach_the_page():
    scripts = []
    page = type('Page', (), {'runJavaScript': lambda self, *args: scripts.append(args)})()
    bridge = MapJavaScript(lambda: page, lambda: True)
    callback = lambda value: None
    bridge.call('setMapCenter', -22., -40., 15, callback=callback)
    bridge.call('updateDroneStatus', False, True)
    assert scripts[0] == ('setMapCenter(-22.0, -40.0, 15);', callback)
    assert scripts[1] == ('updateDroneStatus(false, true);',)


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_coordinate_never_reaches_javascript(value):
    bridge = MapJavaScript(lambda: pytest.fail('Página não deveria ser acessada'), lambda: True)
    with pytest.raises(ValueError):
        bridge.call('addHomeMarker', value, 0.)


def test_unknown_javascript_function_is_rejected():
    bridge = MapJavaScript(lambda: None, lambda: True)
    with pytest.raises(ValueError):
        bridge.call('arbitraryCode')
