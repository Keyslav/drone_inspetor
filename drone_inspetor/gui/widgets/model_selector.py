"""Seleção de redes separada do vídeo; modelos ativos vêm da confirmação ROS."""

import json
from PyQt6.QtCore import QObject, QTimer, Qt
from PyQt6.QtWidgets import (
    QComboBox, QDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QScrollArea, QTabWidget, QVBoxLayout, QWidget,
)
from .cockpit import DASHBOARD_STYLE
from ..logging import gui_log_error


class ModelDetails(QGroupBox):
    """Metadados do candidato; selecionar não significa que ele já está ativo."""

    def __init__(self, title):
        super().__init__(title)
        layout = QFormLayout(self)
        layout.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self.field_labels = {}
        for key, caption in (
            ('name', 'Nome'), ('file_name', 'Arquivo'), ('availability', 'Disponibilidade'),
            ('model', 'Arquitetura'), ('type', 'Tarefa'), ('dataset', 'Dataset'),
            ('classes', 'Classes'), ('validation_warning', 'Observações'),
            ('storage_directory', 'Pasta dos pesos'),
        ):
            value = QLabel('—')
            value.setWordWrap(True)
            value.setMinimumWidth(0)
            value.setTextFormat(Qt.TextFormat.PlainText)
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            layout.addRow(caption, value)
            self.field_labels[key] = value

    def set_model(self, model):
        values = dict(model)
        available = model.get('available')
        values['availability'] = ('Arquivo disponível' if available is True else
                                  'Arquivo ausente' if available is False else 'Não informada')
        if available and model.get('size_bytes') is not None:
            values['availability'] += f" · {model['size_bytes'] / 1024**2:.1f} MiB"
        for key, label in self.field_labels.items():
            value = values.get(key) or '—'
            if isinstance(value, list):
                value = ', '.join(map(str, value))
            label.setText(str(value))


class ModelSelector(QObject):
    """Mantém catálogo, rascunho e modelos ativos como estados distintos."""

    def __init__(self, signals, parent=None):
        super().__init__(parent)
        self.signals = signals
        self.dialog = None
        self.models = {'equipment': [], 'anomaly': []}
        self.active = ('', '')
        self.selected = ['', '']
        self.pending = None
        self._polls = 0
        self._dirty = False
        self.dropdowns, self.details, self.filters = {}, {}, {}
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self._poll_confirmation)
        signals.models_received.connect(self.update_models)

    def open(self, parent=None):
        if self.dialog is None:
            self._create_dialog(parent)
        if not self.dialog.isVisible() and self.pending is None:
            self.selected = list(self.active)
            self._dirty = False
            self._refresh_lists()
        self.dialog.show()
        self.dialog.raise_()
        self.dialog.activateWindow()
        self.signals.models_requested.emit()

    def _create_dialog(self, parent):
        self.dialog = QDialog(parent)
        self.dialog.setWindowTitle('Redes de visão computacional')
        self.dialog.resize(700, 650)
        self.dialog.setMinimumSize(460, 400)
        self.dialog.setStyleSheet(DASHBOARD_STYLE + '''
            QComboBox, QLineEdit { background: #111c2e; color: #e6edf5;
                border: 1px solid #30425b; border-radius: 6px; padding: 8px; }
            QGroupBox { border: 1px solid #26354b; border-radius: 8px;
                margin-top: 12px; padding-top: 15px; }
            QTabBar::tab { padding: 10px 18px; background: #111c2e; }
            QTabBar::tab:selected { color: #4de0c1; border-bottom: 2px solid #4de0c1; }
            QPushButton:disabled, QPushButton#primary:disabled {
                color: #68768a; background: #17263c; }
        ''')
        layout = QVBoxLayout(self.dialog)
        layout.setContentsMargins(18, 18, 18, 18)
        heading = QLabel('Selecionar redes CV')
        heading.setObjectName('heading')
        layout.addWidget(heading)
        self.active_label = QLabel()
        self.active_label.setWordWrap(True)
        self.active_label.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.active_label)
        tabs = QTabWidget()
        for kind, caption in (('equipment', 'Equipamentos'), ('anomaly', 'Anomalias')):
            page = QWidget()
            body = QVBoxLayout(page)
            search = QLineEdit()
            search.setPlaceholderText('Filtrar por nome, arquivo ou classe…')
            combo = QComboBox()
            combo.setMinimumWidth(0)
            combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
            combo.setMinimumContentsLength(16)
            details = ModelDetails('Modelo selecionado')
            self.filters[kind], self.dropdowns[kind], self.details[kind] = search, combo, details
            body.addWidget(search)
            body.addWidget(combo)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(details)
            body.addWidget(scroll, 1)
            tabs.addTab(page, caption)
            search.textChanged.connect(lambda _, k=kind: self._populate(k))
            combo.currentIndexChanged.connect(lambda _, k=kind: self._selected(k))
        layout.addWidget(tabs, 1)
        self.status = QLabel('Aguardando catálogo do nó CV…')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        buttons = QHBoxLayout()
        refresh = QPushButton('Atualizar catálogo')
        refresh.clicked.connect(self.signals.models_requested.emit)
        buttons.addWidget(refresh)
        buttons.addStretch()
        self.apply_button = QPushButton('Aplicar seleção')
        self.apply_button.setObjectName('primary')
        self.apply_button.clicked.connect(self.apply)
        buttons.addWidget(self.apply_button)
        close = QPushButton('Fechar')
        close.clicked.connect(self.dialog.close)
        buttons.addWidget(close)
        layout.addLayout(buttons)
        self._refresh_lists()

    def _populate(self, kind):
        index = 0 if kind == 'equipment' else 1
        combo = self.dropdowns[kind]
        query = self.filters[kind].text().casefold()
        combo.blockSignals(True)
        combo.clear()
        for model in self.models[kind]:
            if query and query not in str(model).casefold():
                continue
            missing = model.get('available') is False
            combo.addItem(model.get('name', model['file_name']) + (' · arquivo ausente' if missing else ''), model['file_name'])
            if missing:
                combo.model().item(combo.count() - 1).setEnabled(False)
        combo.setCurrentIndex(combo.findData(self.selected[index]))
        combo.blockSignals(False)
        self._update_details(kind)
        self._update_apply()

    def _selected(self, kind):
        index = 0 if kind == 'equipment' else 1
        self.selected[index] = self.dropdowns[kind].currentData() or ''
        self._dirty = True
        self._update_details(kind)
        self._update_apply()

    def _update_details(self, kind):
        filename = self.dropdowns[kind].currentData()
        model = next((m for m in self.models[kind] if m['file_name'] == filename), {})
        self.details[kind].set_model(model)

    def _refresh_lists(self):
        if self.dialog is None:
            return
        self.active_label.setText(f'Em uso · Equipamentos: {self.active[0] or "nenhum"}\n'
                                  f'Anomalias: {self.active[1] or "nenhum"}')
        for kind in self.models:
            self._populate(kind)

    def _update_apply(self):
        if not hasattr(self, 'apply_button'):
            return
        pair = tuple(self.dropdowns[k].currentData() or '' for k in self.models)
        changed = any(name and name != current for name, current in zip(pair, self.active))
        valid = all(not name or any(m['file_name'] == name and m.get('available') is not False
                                   for m in self.models[kind])
                    for kind, name in zip(self.models, pair))
        self.apply_button.setEnabled(changed and valid and self.pending is None)

    def update_models(self, data):
        try:
            entries = json.loads(data.get('models_data_json', '[]'))
            if not isinstance(entries, list) or not all(isinstance(m, dict) and m.get('file_name') for m in entries):
                raise ValueError('Formato de catálogo inválido')
        except (ValueError, TypeError) as error:
            gui_log_error('ModelSelector', str(error))
            if self.dialog:
                self.status.setText('Não foi possível ler o catálogo. Tente atualizar.')
            return
        self.models = {kind: [m for m in entries if m.get('object_type') == kind] for kind in self.models}
        self.active = (data.get('current_object_model', ''), data.get('current_anomaly_model', ''))
        confirmed = self.pending is not None and self.pending == self.active
        if confirmed:
            self.pending = None
            self.timer.stop()
            self._dirty = False
        if not self._dirty:
            self.selected = list(self.active)
        if self.dialog:
            if confirmed:
                self.status.setText('Redes ativas confirmadas pelo nó CV.')
            elif self.pending is None:
                self.status.setText('Selecione os modelos e aplique. Fechar não altera as redes em uso.')
            self._refresh_lists()

    def apply(self):
        if not self.apply_button.isEnabled():
            return
        pair = tuple(self.dropdowns[k].currentData() or '' for k in self.models)
        # Campo vazio preserva o modelo da categoria no protocolo atual.
        self.pending = tuple(name or current for name, current in zip(pair, self.active))
        self._polls = 0
        self.status.setText('Seleção enviada. Aguardando confirmação do nó CV…')
        self.signals.send_model_selection(*pair)
        self._update_apply()
        if self.pending is not None:
            self.timer.start()

    def _poll_confirmation(self):
        self._polls += 1
        if self._polls >= 15:
            self.timer.stop()
            self.pending = None
            self.status.setText('Troca ainda não confirmada. Atualize o catálogo e consulte os logs do nó CV.')
            self._update_apply()
        self.signals.models_requested.emit()

    def close(self):
        self.timer.stop()
        if self.dialog:
            self.dialog.close()
