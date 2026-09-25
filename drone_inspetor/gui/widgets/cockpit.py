"""Componentes visuais do dashboard; recebem snapshots sem acessar ROS."""

import math
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout

DASHBOARD_STYLE = '''
QWidget { background: #0b1321; color: #e6edf5; font-family: 'Inter', 'Noto Sans', sans-serif; font-size: 13px; }
QFrame#panel, QFrame#stat { background: #111c2e; border: 1px solid #26354b; border-radius: 12px; }
QFrame#panel QLabel, QFrame#stat QLabel { background: transparent; border: none; }
QLabel#brand { color: #4de0c1; font-size: 12px; font-weight: 700; letter-spacing: 2px; }
QLabel#heading { font-size: 24px; font-weight: 700; }
QLabel#muted, QLabel#eyebrow { color: #93a4bb; font-size: 11px; }
QLabel#panelTitle { font-size: 13px; font-weight: 600; }
QLabel#statValue { font-size: 19px; font-weight: 600; }
QPushButton { background: #17263c; border: 1px solid #30425b; border-radius: 7px; padding: 9px 14px; color: #e6edf5; }
QPushButton:hover { background: #203550; border-color: #4de0c1; }
QPushButton:focus { border-color: #4de0c1; }
QPushButton#primary { background: #4de0c1; color: #0b1321; font-weight: 600; border: none; }
QPushButton#expand { border: none; background: transparent; color: #93a4bb; padding: 3px 6px; }
QScrollArea { border: none; }
QScrollBar:vertical { background: #0b1321; width: 8px; }
QScrollBar::handle:vertical { background: #30425b; border-radius: 4px; min-height: 30px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
'''


class Panel(QFrame):
    """Cabeçalho discreto e ação explícita para ampliar o conteúdo."""

    def __init__(self, title, subtitle='', on_expand=None):
        super().__init__()
        self.setObjectName('panel')
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(12, 10, 12, 12)
        self.body.setSpacing(8)
        header = QHBoxLayout()
        self.header = header
        self.title = QLabel(title)
        self.title.setObjectName('panelTitle')
        header.addWidget(self.title)
        header.addStretch()
        if on_expand:
            button = QPushButton('Ampliar ↗')
            button.setObjectName('expand')
            button.setAccessibleName(f'Ampliar {title}')
            button.clicked.connect(on_expand)
            header.addWidget(button)
        self.body.addLayout(header)
        if subtitle:
            label = QLabel(subtitle)
            label.setObjectName('muted')
            label.setWordWrap(True)
            self.body.addWidget(label)


def numeric(value, suffix):
    return f'{value:.1f} {suffix}' if isinstance(value, (int, float)) and math.isfinite(value) else '—'


class FlightSummary(QFrame):
    """Só apresenta valores recentes; tempo sem mensagens também atualiza o estado."""

    def __init__(self, store):
        super().__init__()
        self.store = store
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(10)
        self.cards, self.values, self.notes = [], [], []
        for heading in ('ESTADO DO DRONE', 'MODO PX4', 'VELOCIDADE', 'BATERIA'):
            card = QFrame()
            card.setObjectName('stat')
            layout = QVBoxLayout(card)
            layout.setContentsMargins(14, 10, 14, 10)
            title, value, note = QLabel(heading), QLabel('—'), QLabel('Aguardando dados')
            title.setObjectName('eyebrow')
            value.setObjectName('statValue')
            note.setObjectName('muted')
            value.setWordWrap(True)
            for label in (title, value, note):
                layout.addWidget(label)
            self.cards.append(card)
            self.values.append(value)
            self.notes.append(note)
        self.reflow(False)
        self.refresh()

    def reflow(self, compact):
        columns = 2 if compact else 4
        for index, card in enumerate(self.cards):
            self.grid.addWidget(card, index // columns, index % columns)
        for column in range(4):
            self.grid.setColumnStretch(column, 1 if column < columns else 0)

    def refresh(self):
        samples = self.store.snapshot()
        data = {key: sample.values if sample.health == 'live' else {} for key, sample in samples.items()}
        velocity = [data['local'].get(key) for key in ('vx', 'vy', 'vz')]
        speed = (math.sqrt(sum(v*v for v in velocity)) if all(
            isinstance(v, (int, float)) and math.isfinite(v) for v in velocity) else None)
        battery = data['battery'].get('remaining')
        battery = battery * 100 if data['battery'].get('connected') and isinstance(battery, (int, float)) and 0 <= battery <= 1 else None
        values = (data['drone'].get('state_name', '—').replace('_', ' ').capitalize(),
                  data['status'].get('nav_state_name', '—').replace('_', ' ').capitalize(),
                  numeric(speed, 'm/s'), numeric(battery, '%'))
        for index, key in enumerate(('drone', 'status', 'local', 'battery')):
            sample = samples[key]
            self.values[index].setText(values[index])
            text = {'live': 'Recebendo', 'stale': 'Dados desatualizados', 'waiting': 'Aguardando dados'}[sample.health]
            self.notes[index].setText(text)
            self.notes[index].setStyleSheet('color: ' + ('#4de0c1' if sample.health == 'live' else '#93a4bb') + ';')
