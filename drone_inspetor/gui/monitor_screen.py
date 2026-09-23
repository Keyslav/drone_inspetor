"""Monitor de telemetria: atualização limitada, sem comandos de voo."""

from collections import deque
from collections.abc import Mapping
import math
import time

from PyQt6.QtCore import QPointF, Qt, QTimer
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import (
    QAbstractItemView, QFrame, QGridLayout, QHBoxLayout, QHeaderView,
    QLabel, QLineEdit, QProgressBar, QScrollArea, QSplitter, QTabWidget,
    QTableWidget, QTableWidgetItem, QTreeWidget, QTreeWidgetItem, QVBoxLayout,
    QWidget,
)

from drone_inspetor.gui.presentation.telemetry import COORDINATE_NOTES, MONITOR_TOPICS


COLORS = {'live': '#6ee7b7', 'stale': '#fbbf24', 'waiting': '#94a3b8'}
HEALTH = {'live': 'Recebendo', 'stale': 'Desatualizado', 'waiting': 'Sem dados'}
STYLE = """
QWidget { background: #0b1220; color: #e2e8f0; font-family: 'DejaVu Sans'; font-size: 12px; }
QLabel { background: transparent; }
QFrame#panel { background: #131e30; border: 1px solid #273449; border-radius: 10px; }
QLabel#eyebrow { color: #94a3b8; font-size: 11px; }
QLabel#value { font-size: 24px; font-weight: 600; color: #f8fafc; }
QLabel#title { font-size: 25px; font-weight: 600; }
QTabWidget::pane { border: 0; }
QTabBar::tab { padding: 12px 20px; background: #131e30; color: #94a3b8; }
QTabBar::tab:selected { color: #67e8f9; border-bottom: 2px solid #22d3ee; }
QHeaderView::section { background: #1e293b; border: 0; padding: 9px; color: #cbd5e1; }
QTableWidget, QTreeWidget { background: #101a2b; gridline-color: #243247; border: 0; }
QTableWidget::item, QTreeWidget::item { padding: 6px; }
QTableWidget::item:selected, QTreeWidget::item:selected { background: #164e63; }
QLineEdit { background: #131e30; border: 1px solid #334155; padding: 10px; border-radius: 6px; }
QProgressBar { border: 0; background: #243247; border-radius: 5px;
               height: 18px; text-align: center; }
QProgressBar::chunk { background: #0e7490; border-radius: 5px; }
"""


def number(value, unit='', digits=2):
    """Não transforma ausência ou NaN em zero operacional."""
    if not isinstance(value, (float, int)) or not math.isfinite(value):
        return '—'
    return f'{value:.{digits}f}{" " + unit if unit else ""}'


def magnitude(values):
    """Norma de um vetor completo e finito, ou ausência explícita."""
    if len(values) != 3 or any(not isinstance(v, (int, float)) or not math.isfinite(v)
                               for v in values):
        return None
    return math.sqrt(sum(v * v for v in values))


def vector(values, unit='m'):
    return ' / '.join(number(value, digits=2) for value in values) + f' {unit}'


def human(value):
    return str(value).replace('_', ' ').capitalize() if value else '—'


def boolean(value):
    return 'Sim' if value is True else 'Não' if value is False else '—'


class SpeedHistory(QWidget):
    """Últimos 60 segundos; lacunas de dados interrompem as curvas."""

    def __init__(self):
        super().__init__()
        self.samples = deque(maxlen=310)
        self.setMinimumHeight(160)

    def append(self, now, measured, reference):
        self.samples.append((now, measured, reference))
        while self.samples and now - self.samples[0][0] > 60:
            self.samples.popleft()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        left, top, width, height = 45., 23., max(1., self.width() - 62.), self.height() - 52.
        speeds = (v for s in self.samples for v in s[1:] if v is not None)
        maximum = max(1., max(speeds, default=1.) * 1.2)
        for fraction in (0., .5, 1.):
            y = top + (1 - fraction) * height
            painter.setPen(QColor('#273449'))
            painter.drawLine(QPointF(left, y), QPointF(left + width, y))
            painter.setPen(QColor('#94a3b8'))
            painter.drawText(0, int(y + 4), f'{maximum * fraction:.1f}')
        painter.drawText(0, 12, 'm/s')
        painter.drawText(int(left), self.height() - 5, '−60 s')
        painter.drawText(int(left + width - 40), self.height() - 5, 'agora')
        if not self.samples:
            return
        now = self.samples[-1][0]
        for column, color in ((1, '#67e8f9'), (2, '#c4b5fd')):
            painter.setPen(QPen(QColor(color), 2))
            previous = None
            previous_time = None
            for stamp, *values in self.samples:
                value = values[column - 1]
                if value is None:
                    previous = None
                    continue
                point = QPointF(left + width * (1 - (now - stamp) / 60),
                                top + height * (1 - value / maximum))
                if previous is not None and stamp - previous_time < 1.:
                    painter.drawLine(previous, point)
                previous, previous_time = point, stamp


class MonitorWindow(QWidget):
    """Consulta snapshots do transporte sem executar ROS na thread gráfica."""

    def __init__(self, store, parent=None, clock=time.monotonic):
        super().__init__(parent, Qt.WindowType.Window)
        self.store, self.clock = store, clock
        self.setWindowTitle('Monitor do drone · Drone Inspetor')
        self.resize(1180, 900)
        self.setMinimumSize(820, 640)
        self.setStyleSheet(STYLE)
        self.cards, self.fields = {}, {}
        self.keys = [topic.key for topic in MONITOR_TOPICS]
        self.snapshots = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        title = QLabel('Monitor do drone')
        title.setObjectName('title')
        layout.addWidget(title)
        layout.addWidget(QLabel('Estado de voo, PX4 e missão em uma única tela · Somente leitura'))
        self.banner = QLabel()
        self.banner.setWordWrap(True)
        layout.addWidget(self.banner)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)
        self._overview()
        self._topics()
        footer = QLabel('Idade e Hz medidos na recepção local. '
                        'Dados desatualizados não representam o estado atual.')
        footer.setObjectName('eyebrow')
        footer.setWordWrap(True)
        layout.addWidget(footer)
        self.timer = QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh)
        self.refresh()

    def _panel(self, title, rows):
        frame = QFrame()
        frame.setObjectName('panel')
        grid = QGridLayout(frame)
        grid.setContentsMargins(18, 16, 18, 16)
        grid.setVerticalSpacing(12)
        heading = QLabel(title)
        heading.setStyleSheet('font-size: 15px; font-weight: 600;')
        grid.addWidget(heading, 0, 0, 1, 2)
        for index, (key, caption) in enumerate(rows, 1):
            label, value = QLabel(caption), QLabel('—')
            label.setObjectName('eyebrow')
            value.setWordWrap(True)
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            grid.addWidget(label, index, 0)
            grid.addWidget(value, index, 1)
            self.fields[key] = value
        grid.setColumnStretch(1, 1)
        return frame

    def _overview(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 18, 0, 0)
        cards = QHBoxLayout()
        for key, title in (('drone', 'ESTADO DO DRONE'), ('status', 'MODO PX4'),
                           ('speed', 'VELOCIDADE'), ('battery', 'BATERIA')):
            frame = QFrame()
            frame.setObjectName('panel')
            box = QVBoxLayout(frame)
            box.setContentsMargins(16, 14, 16, 14)
            eyebrow, value, subtitle = QLabel(title), QLabel('—'), QLabel('Sem dados')
            eyebrow.setObjectName('eyebrow')
            value.setObjectName('value')
            value.setWordWrap(True)
            subtitle.setObjectName('eyebrow')
            subtitle.setWordWrap(True)
            for widget in (eyebrow, value, subtitle):
                box.addWidget(widget)
            self.cards[key] = (value, subtitle)
            cards.addWidget(frame, 1)
        layout.addLayout(cards)
        grid = QGridLayout()
        grid.addWidget(self._panel('Voo e segurança', [
            ('armed', 'Motores'), ('landed', 'No solo'), ('failsafe', 'Failsafe PX4'),
            ('moving', 'Trajetória'), ('detour', 'Desvio ativo'), ('yaw', 'Rumo')]), 0, 0)
        grid.addWidget(self._panel('Missão', [
            ('mission_name', 'Missão atual'), ('mission_state', 'Etapa'),
            ('mission_active', 'Em execução'), ('waypoint', 'Ponto de inspeção'),
            ('object', 'Objeto alvo'), ('cancel', 'Cancelamento solicitado')]), 0, 1)
        grid.addWidget(self._panel('Posição e movimento', [
            ('position', 'Norte / Leste / Abaixo'), ('height', 'Altura sobre origem local'),
            ('velocity', 'Velocidade N / L / A'), ('climb', 'Velocidade de subida'),
            ('valid', 'Estimativa PX4'), ('gps', 'GPS informado pelo drone')]), 1, 0)
        grid.addWidget(self._panel('Referência e auxiliares', [
            ('reference', 'Posição desejada N / L / A'), ('reference_speed', 'Velocidade desejada'),
            ('reference_accel', 'Aceleração desejada'), ('lidar', 'LiDAR horizontal'),
            ('down', 'LiDAR inferior'), ('depth', 'Profundidade')]), 1, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        layout.addLayout(grid)
        self.progress = QProgressBar()
        layout.addWidget(self.progress)
        layout.addWidget(QLabel('Velocidade · <span style="color:#67e8f9">medida</span> / '
                                '<span style="color:#c4b5fd">referência</span> · últimos 60 s'))
        self.history = SpeedHistory()
        layout.addWidget(self.history)
        layout.addWidget(QLabel('N / L / A: Norte, Leste e Abaixo (NED). '
                                'Altura e subida são positivas para cima.'))
        layout.addStretch()
        scroll.setWidget(content)
        self.tabs.addTab(scroll, 'Visão geral')

    def _topics(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(QLabel('Selecione um tópico para inspecionar seus campos. '
                                'Leituras antigas ficam identificadas.'))
        splitter = QSplitter(Qt.Orientation.Vertical)
        self.topic_table = QTableWidget(len(self.keys), 5)
        self.topic_table.setHorizontalHeaderLabels(
            ['Fonte', 'Recepção', 'Idade', 'Hz', 'Tópico ROS 2'])
        self.topic_table.verticalHeader().hide()
        self.topic_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.topic_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.topic_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        for row, topic in enumerate(MONITOR_TOPICS):
            for column, text in enumerate((topic.label, 'Sem dados', '—', '—', topic.topic)):
                self.topic_table.setItem(row, column, QTableWidgetItem(text))
            self.topic_table.setRowHeight(row, 36)
        self.topic_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents)
        self.topic_table.horizontalHeader().setStretchLastSection(True)
        splitter.addWidget(self.topic_table)
        details = QWidget()
        detail_layout = QVBoxLayout(details)
        self.detail_status = QLabel()
        self.detail_status.setWordWrap(True)
        detail_layout.addWidget(self.detail_status)
        self.coordinate_note = QLabel()
        self.coordinate_note.setWordWrap(True)
        self.coordinate_note.setObjectName('eyebrow')
        detail_layout.addWidget(self.coordinate_note)
        self.search = QLineEdit()
        self.search.setPlaceholderText('Filtrar campos: state, velocity, failsafe…')
        self.search.textChanged.connect(self._details)
        detail_layout.addWidget(self.search)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(['Campo', 'Último valor recebido'])
        self.tree.setColumnWidth(0, 340)
        detail_layout.addWidget(self.tree)
        splitter.addWidget(details)
        splitter.setSizes([350, 300])
        layout.addWidget(splitter)
        self.topic_table.itemSelectionChanged.connect(self._details)
        self.topic_table.selectRow(0)
        self.tabs.addTab(tab, 'Tópicos e campos')

    def _details(self):
        row = self.topic_table.currentRow()
        if row < 0 or not self.snapshots:
            return
        sample = self.snapshots[self.keys[row]]
        self.coordinate_note.setText(COORDINATE_NOTES.get(self.keys[row], ''))
        self.detail_status.setText(
            f'{sample.topic} · {HEALTH[sample.health]} · idade {number(sample.age, "s")}')
        self.detail_status.setStyleSheet(f'color: {COLORS[sample.health]};')
        scroll = self.tree.verticalScrollBar().value()
        query = self.search.text().casefold()
        rows = []
        for key, value in sample.values.items():
            if query in key.casefold():
                text = str(dict(value)) if isinstance(value, Mapping) else str(value)
                rows.append((key, text))
        while self.tree.topLevelItemCount() > len(rows):
            self.tree.takeTopLevelItem(self.tree.topLevelItemCount() - 1)
        for index, (key, text) in enumerate(rows):
            item = self.tree.topLevelItem(index)
            if item is None:
                self.tree.addTopLevelItem(QTreeWidgetItem([key, text]))
            else:
                item.setText(0, key)
                item.setText(1, text)
        self.tree.verticalScrollBar().setValue(scroll)

    def refresh(self):
        now = self.clock()
        self.snapshots = self.store.snapshot(now=now)
        data = {key: dict(sample.values) if sample.health == 'live' else {}
                for key, sample in self.snapshots.items()}
        drone, status, local = data['drone'], data['status'], data['local']
        battery, mission, reference = data['battery'], data['mission'], data['setpoint']
        velocity_valid = local.get('v_xy_valid') and local.get('v_z_valid')
        velocity = [local.get(k) if velocity_valid else None for k in ('vx', 'vy', 'vz')]
        speed = magnitude(velocity)
        ref_velocity = reference.get('velocity', (None,) * 3)
        ref_speed = magnitude(ref_velocity)
        remaining = battery.get('remaining', -1)
        battery_value = (number(remaining * 100, '%', 0)
                         if battery.get('connected') and 0 <= remaining <= 1 else '—')
        for key, text, source in (
            ('drone', human(drone.get('state_name')), 'drone'),
            ('status', human(status.get('nav_state_name')), 'status'),
            ('speed', number(speed, 'm/s'), 'local'), ('battery', battery_value, 'battery'),
        ):
            sample = self.snapshots[source]
            self.cards[key][0].setText(text)
            self.cards[key][1].setText(f'{HEALTH[sample.health]} · {number(sample.age, "s", 1)}')
            self.cards[key][1].setStyleSheet(f'color: {COLORS[sample.health]};')
        if battery.get('connected'):
            voltage = battery.get('voltage_v', 0)
            current = battery.get('current_a', -1)
            self.cards['battery'][1].setText(
                f'{number(voltage if voltage > 0 else None, "V", 1)} · '
                f'{number(current if current >= 0 else None, "A", 1)}')
        if status.get('failsafe'):
            banner, color = 'ATENÇÃO · Failsafe ativo no PX4', '#fca5a5'
        elif battery.get('warning', 0) > 0:
            banner, color = 'ATENÇÃO · Alerta de bateria informado pelo PX4', '#fbbf24'
        elif not status or not local:
            banner = 'Telemetria PX4 ausente ou desatualizada · confira a aba Tópicos e campos'
            color = '#fbbf24'
        else:
            banner = ('Telemetria PX4 recebida · disponibilidade das demais fontes '
                      'na aba Tópicos e campos')
            color = '#6ee7b7'
        self.banner.setText(banner)
        self.banner.setStyleSheet(f'color: {color}; padding: 8px 0;')
        position = [local.get('x') if local.get('xy_valid') else None,
                    local.get('y') if local.get('xy_valid') else None,
                    local.get('z') if local.get('z_valid') else None]
        z, vz = position[2], velocity[2]
        total = mission.get('total_pontos_de_inspecao', 0)
        index = mission.get('ponto_de_inspecao_indice_atual', -1)
        waypoint = f'{index + 1} de {total}' if total > 0 and 0 <= index < total else '—'
        values = {
            'armed': {'ARMED': 'Armados', 'DISARMED': 'Desarmados'}.get(
                status.get('arming_state_name'), human(status.get('arming_state_name'))),
            'landed': boolean(drone.get('is_landed')), 'failsafe': boolean(status.get('failsafe')),
            'moving': boolean(drone.get('is_on_trajectory')),
            'detour': boolean(drone.get('has_trajectory_adjusted')),
            'yaw': number(drone.get('current_yaw_deg'), '°', 1),
            'mission_name': mission.get('mission_name') or '—',
            'mission_state': human(mission.get('state_name')),
            'mission_active': boolean(mission.get('on_mission')),
            'waypoint': waypoint, 'object': mission.get('objeto_alvo') or '—',
            'cancel': boolean(mission.get('cancel_mission')), 'position': vector(position),
            'height': number(-z if z is not None else None, 'm'),
            'velocity': vector(velocity, 'm/s'),
            'climb': number(-vz if vz is not None else None, 'm/s'),
            'valid': f'XY: {boolean(local.get("xy_valid"))} · Z: {boolean(local.get("z_valid"))}',
            'gps': (f'{number(drone.get("current_latitude"), digits=6)}, '
                    f'{number(drone.get("current_longitude"), digits=6)}'),
            'reference': vector(reference.get('position', (None,) * 3)),
            'reference_speed': vector(ref_velocity, 'm/s'),
            'reference_accel': vector(reference.get('acceleration', (None,) * 3), 'm/s²'),
        }
        for key in ('lidar', 'down', 'depth'):
            sample = self.snapshots[key]
            distance = data[key].get('minimum_distance')
            suffix = f' · retorno mais próximo: {number(distance, "m")}' if data[key] else ''
            values[key] = HEALTH[sample.health] + suffix
        for key, text in values.items():
            self.fields[key].setText(str(text))
        self.progress.setRange(0, max(1, total))
        self.progress.setValue(index + 1 if mission.get('on_mission') and waypoint != '—' else 0)
        self.progress.setFormat(f'Ponto atual: {waypoint}' if mission.get('on_mission')
                                else 'Sem missão ativa recebida')
        self.history.append(now, speed, ref_speed)
        for row, key in enumerate(self.keys):
            sample = self.snapshots[key]
            hz = number(sample.hz, digits=1) if sample.age is not None else '—'
            for column, text in ((1, HEALTH[sample.health]), (2, number(sample.age, 's', 1)),
                                 (3, hz), (4, sample.topic)):
                item = self.topic_table.item(row, column)
                item.setText(text)
                item.setForeground(QColor(COLORS[sample.health]))
        self._details()

    def showEvent(self, event):
        self.refresh()
        self.timer.start()
        super().showEvent(event)

    def hideEvent(self, event):
        self.timer.stop()
        super().hideEvent(event)
