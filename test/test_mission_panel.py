"""Contratos de apresentação: largura compacta e comandos apenas por ação do usuário."""

import json
import os
from types import SimpleNamespace

import pytest
from PyQt6.QtWidgets import QApplication

from drone_inspetor.gui.controles import ControlesManager
from drone_inspetor.gui.mission import MissionManager


@pytest.fixture(scope='module')
def qt_app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    app = QApplication.instance() or QApplication([])
    yield app
    app.processEvents()


def test_mission_snapshot_is_visible_and_details_are_collapsed(qt_app):
    manager = MissionManager()
    # O snapshot pode chegar antes da montagem dos widgets.
    manager.update_state({
        'state_name': 'EXECUTANDO_INSPECIONANDO_ESCANEANDO',
        'mission_name': 'Flare', 'ponto_de_inspecao_indice_atual': 1,
        'total_pontos_de_inspecao': 4, 'objeto_alvo': 'flare',
        'tipos_anomalia': ['mild_corrosion'],
    })
    panel = manager.setup_b2_mission()
    panel.resize(260, panel.sizeHint().height())
    panel.show()
    qt_app.processEvents()
    assert manager.state_label.text() == 'Escaneando equipamento'
    assert manager.progress_label.text() == 'Ponto atual · 2 de 4'
    assert manager.progress_bar.value() == 2
    assert not manager.progress_bar.isHidden()
    assert manager.mission_tree.isHidden()
    assert panel.minimumSizeHint().width() <= 260
    manager.details_toggle.click()
    assert not manager.mission_tree.isHidden()
    assert manager.mission_tree.currentItem().toolTip(0) == manager.current_state
    panel.close()


def test_unknown_state_is_preserved_in_summary(qt_app):
    manager = MissionManager()
    panel = manager.setup_b2_mission()
    manager.update_state('NOVO_ESTADO')
    assert manager.state_label.text() == 'Novo estado'
    assert manager.state_label.toolTip() == 'NOVO_ESTADO'
    assert not manager.mission_tree.selectedItems()
    assert manager.progress_bar.isHidden()
    assert manager.progress_label.isHidden()
    panel.close()


def test_compact_controls_preview_does_not_start_mission(qt_app):
    commands, previews = [], []
    controls = ControlesManager(
        SimpleNamespace(send_mission_command=commands.append),
        SimpleNamespace(mission_selected=SimpleNamespace(emit=previews.append)),
    )
    long_name = 'Inspeção da plataforma e dos equipamentos de combustão'
    controls.set_missions({'Flare': {}, long_name: {}})
    panel = controls.setup_b3_controls()
    panel.resize(260, panel.sizeHint().height())
    panel.show()
    controls.inspection_selector.setCurrentText(long_name)
    qt_app.processEvents()
    assert commands == []
    assert previews[-1] == long_name
    assert panel.minimumSizeHint().width() <= 260
    controls.start_button.click()
    controls.cancel_button.click()
    assert [json.loads(command) for command in commands] == [
        {'command': 'iniciar_missao', 'mission': long_name},
        {'command': 'cancelar_missao'},
    ]
    panel.close()


def test_controls_preserve_selection_on_reload_and_disable_empty_list(qt_app):
    commands = []
    controls = ControlesManager(SimpleNamespace(send_mission_command=commands.append))
    controls.set_missions({'Flare': {}, 'Tubulação': {}})
    panel = controls.setup_b3_controls()
    controls.inspection_selector.setCurrentText('Tubulação')
    controls.set_missions({'Flare': {}, 'Tubulação': {}, 'Outro': {}})
    assert controls.inspection_selector.currentText() == 'Tubulação'
    controls.set_missions({})
    assert not controls.start_button.isEnabled()
    controls.iniciar_missao()
    assert commands == []
    panel.close()
