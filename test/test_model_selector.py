"""Escolha é um rascunho até a confirmação do catálogo recebido pelo ROS."""

import json
from PyQt6.QtWidgets import QApplication
from drone_inspetor.signals.dashboard_signals import CVSignals
from drone_inspetor.gui.widgets.model_selector import ModelSelector
from drone_inspetor.nodes.cv_node.model_registry import ModelRegistry, resolve_models_directory


def catalog(current='one.pt'):
    return dict(models_data_json=json.dumps([
        dict(file_name='one.pt', name='Primeiro', object_type='equipment', available=True),
        dict(file_name='two.pt', name='Segundo', object_type='equipment', available=True),
        dict(file_name='absent.pt', name='Ausente', object_type='equipment', available=False),
        dict(file_name='anom.pt', name='Anomalia', object_type='anomaly', available=True),
    ]), current_object_model=current, current_anomaly_model='anom.pt')


def test_popup_drafts_cancel_and_wait_for_real_confirmation():
    app = QApplication.instance() or QApplication([])
    signals = CVSignals()
    sent = []
    signals.send_model_selection = lambda *pair: sent.append(pair)
    selector = ModelSelector(signals)
    selector.update_models(catalog())
    selector.open()
    combo = selector.dropdowns['equipment']
    combo.setCurrentIndex(combo.findData('two.pt'))
    assert not sent and selector.active[0] == 'one.pt'
    selector.dialog.close()
    selector.open()
    assert combo.currentData() == 'one.pt'  # fechar descarta o rascunho
    combo.setCurrentIndex(combo.findData('two.pt'))
    selector.apply()
    assert sent == [('two.pt', 'anom.pt')]
    assert selector.active[0] == 'one.pt'
    selector.update_models(catalog())  # resposta antiga não confirma a troca
    assert selector.pending == ('two.pt', 'anom.pt')
    selector.update_models(catalog('two.pt'))
    assert selector.pending is None and not selector.timer.isActive()
    assert 'confirmadas' in selector.status.text()
    selector.close()


def test_missing_weights_disabled_and_filter_does_not_publish():
    app = QApplication.instance() or QApplication([])
    signals = CVSignals()
    sent = []
    signals.send_model_selection = lambda *pair: sent.append(pair)
    selector = ModelSelector(signals)
    selector.update_models(catalog())
    selector.open()
    combo = selector.dropdowns['equipment']
    assert not combo.model().item(combo.findData('absent.pt')).isEnabled()
    selector.filters['equipment'].setText('Segundo')
    assert combo.count() == 1
    assert not selector.apply_button.isEnabled()
    combo.setCurrentIndex(0)
    assert selector.apply_button.isEnabled()
    assert not sent
    selector.close()


def test_catalog_metadata_and_external_storage(tmp_path):
    weights = tmp_path / 'pesos'
    weights.mkdir()
    (weights / 'models.json').write_text(json.dumps({'models': [
        dict(file_name='real.pt', object_type='equipment'),
        dict(file_name='missing.pt', object_type='anomaly'),
    ]}))
    (weights / 'real.pt').write_bytes(b'weights')
    assert resolve_models_directory(str(weights), '/share') == weights
    assert resolve_models_directory('', tmp_path) == tmp_path / 'redes_treinadas'
    registry = ModelRegistry(weights)
    first, absent = registry.entries
    assert first['available'] and first['size_bytes'] == 7
    assert first['storage_directory'] == str(weights)
    assert not absent['available'] and absent['size_bytes'] is None
    assert 'available' not in json.loads((weights / 'models.json').read_text())['models'][0]
