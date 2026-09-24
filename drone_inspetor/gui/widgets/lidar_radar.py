"""Radar vetorial leve; escala física e proporção independem do tamanho da tela."""

import math
import time

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, QTimer
from PyQt6.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)


BACKGROUND = '#111c2e'
GRID = '#26354b'
TEXT = '#e6edf5'
MUTED = '#95a7bd'
ACCENT = '#4de0c1'
WARNING = '#f6bc62'
DANGER = '#ff7b86'


def valid_points(vector):
    """Descarta pares incompletos/NaN; ausência de retorno não significa livre."""
    points = []
    if vector is None:
        return points
    for distance, angle in zip(vector[::2], vector[1::2]):
        try:
            distance, angle = float(distance), float(angle)
        except (TypeError, ValueError):
            continue
        if math.isfinite(distance) and distance > 0 and math.isfinite(angle):
            points.append((distance, angle))
    return points


def polar_to_screen(distance, angle, center, scale):
    """FLU: +X à frente e +Y à esquerda; no widget, +Y aponta para baixo."""
    return QPointF(center.x() - distance * math.sin(angle) * scale,
                   center.y() - distance * math.cos(angle) * scale)


class RadarCanvas(QWidget):
    """Recalcula o círculo usando a menor dimensão, sem esticar os dados."""

    def __init__(self, radar):
        super().__init__(radar)
        self.radar = radar
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setToolTip('Vista superior relativa ao drone: frente em 0°, esquerda em +90°.\n'
                        'A escala indica o raio em metros, não o alcance garantido do sensor.')

    def minimumSizeHint(self):
        return QSize(100, 100)

    def sizeHint(self):
        return QSize(300, 260)

    def radar_geometry(self):
        """Reserva bordas para orientação; pixels lógicos mantêm nitidez em HiDPI."""
        center = QPointF(self.width() / 2, self.height() / 2)
        radius = max(1., min(self.width(), self.height()) / 2 - 21.)
        return center, radius

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(BACKGROUND))
        center, radius = self.radar_geometry()
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(GRID), 1))

        for fraction in (.25, .5, .75, 1.):
            painter.drawEllipse(center, radius * fraction, radius * fraction)
        for angle in (0., math.pi / 4, math.pi / 2, 3 * math.pi / 4):
            start = polar_to_screen(-radius, angle, center, 1.)
            end = polar_to_screen(radius, angle, center, 1.)
            painter.drawLine(start, end)

        painter.setPen(QColor(MUTED))
        align = Qt.AlignmentFlag.AlignCenter
        painter.drawText(QRectF(center.x() - 55, center.y() - radius - 21, 110, 18),
                         align, 'FRENTE · 0°')
        painter.drawText(QRectF(center.x() - 40, center.y() + radius + 3, 80, 18),
                         align, 'TRÁS')
        painter.drawText(QRectF(center.x() - radius - 22, center.y() - 10, 18, 20), align, 'E')
        painter.drawText(QRectF(center.x() + radius + 4, center.y() - 10, 18, 20), align, 'D')
        # Duas legendas métricas bastam para leitura em painéis estreitos.
        for fraction in (.5, 1.):
            label = f'{self.radar.display_range * fraction:g} m'
            painter.drawText(QRectF(center.x() + 4, center.y() + radius * fraction - 16,
                                    48, 15), label)

        if self.radar.has_fresh_points():
            painter.setPen(Qt.PenStyle.NoPen)
            for distance, angle in self.radar.points:
                if distance > self.radar.display_range:
                    continue
                color = DANGER if distance <= 1 else WARNING if distance <= 3 else ACCENT
                painter.setBrush(QColor(color))
                point = polar_to_screen(distance, angle, center,
                                        radius / self.radar.display_range)
                painter.drawEllipse(point, 2.4, 2.4)
        # A seta não gira com yaw global: este radar permanece solidário ao corpo.
        painter.setPen(QPen(QColor(BACKGROUND), 2))
        painter.setBrush(QColor(TEXT))
        painter.drawPolygon(QPolygonF([
            center + QPointF(0, -9), center + QPointF(6, 6),
            center + QPointF(0, 3), center + QPointF(-6, 6),
        ]))


class LidarRadar(QWidget):
    """Estado de apresentação; não participa do controle de evasão de obstáculos.

    LidarMSG não possui timestamp: a idade aqui é da recepção na GUI. O lidar_node
    também expira seus scans na origem. Nunca exibimos a última nuvem congelada
    como se ela ainda representasse o entorno atual.
    """

    STALE_SECONDS = 1.5

    def __init__(self, parent=None, clock=time.monotonic):
        super().__init__(parent)
        self._clock = clock
        self.points = []
        self._points_received_at = None
        self._ground_received_at = None
        self.ground_distance = None
        self.display_range = 12.
        self.setStyleSheet(f'''
            QWidget {{ background: {BACKGROUND}; color: {TEXT}; border: none; }}
            QLabel {{ background: transparent; font-size: 11px; }}
            QComboBox {{ border: 1px solid {GRID}; border-radius: 5px;
                        padding: 3px 7px; min-width: 48px; color: {TEXT}; }}
            QComboBox:focus {{ border-color: {ACCENT}; }}
            QComboBox QAbstractItemView {{ background: {BACKGROUND}; color: {TEXT};
                                         selection-background-color: {GRID}; }}
        ''')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(4)
        top = QHBoxLayout()
        self.status_label = QLabel('Aguardando LiDAR')
        self.status_label.setToolTip('Idade desde a recepção na interface.\n'
                                     'Sem retornos não significa ausência de obstáculos.')
        self.status_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        top.addWidget(self.status_label, 1)
        self.range_selector = QComboBox()
        for value in (3, 6, 12):
            self.range_selector.addItem(f'{value} m', value)
        self.range_selector.setCurrentIndex(2)
        self.range_selector.setAccessibleName('Raio de exibição do radar')
        self.range_selector.setToolTip('Raio de exibição. Retornos além desse raio são ocultados.')
        self.range_selector.currentIndexChanged.connect(self._change_range)
        top.addWidget(self.range_selector)
        layout.addLayout(top)
        self.canvas = RadarCanvas(self)
        layout.addWidget(self.canvas, 1)
        self.nearest_label = QLabel('Mín.: —')
        self.nearest_label.setToolTip('Retorno horizontal mais próximo, inclusive fora do raio exibido.')
        self.ground_label = QLabel('Abaixo: —')
        self.ground_label.setToolTip('Distância ao solo/obstáculo abaixo do sensor; não é altitude PX4.')
        bottom = QHBoxLayout()
        bottom.addWidget(self.nearest_label, 1)
        bottom.addWidget(self.ground_label, 1)
        layout.addLayout(bottom)
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self.refresh_status)
        self._timer.start()
        self.refresh_status()

    def _change_range(self, index):
        self.display_range = float(self.range_selector.itemData(index))
        self.canvas.update()

    def _fresh(self, received_at):
        return received_at is not None and self._clock() - received_at <= self.STALE_SECONDS

    def has_fresh_points(self):
        return bool(self.points) and self._fresh(self._points_received_at)

    def set_points(self, vector):
        self.points = valid_points(vector)
        self._points_received_at = self._clock()
        self.refresh_status()
        self.canvas.update()

    def set_ground_distance(self, distance):
        try:
            value = float(distance)
        except (TypeError, ValueError):
            value = math.nan
        self.ground_distance = value if math.isfinite(value) and value >= 0 else None
        self._ground_received_at = self._clock()
        self.refresh_status()

    def refresh_status(self):
        """Usa relógio monotônico; pausa/reset do /clock não mascara a interrupção."""
        stale = self._points_received_at is not None and not self._fresh(self._points_received_at)
        if self._points_received_at is None:
            text, color = 'Aguardando LiDAR', MUTED
        elif stale:
            text, color = 'Recepção atrasada', WARNING
        elif not self.points:
            text, color = 'Sem retornos válidos', MUTED
        else:
            text, color = f'{len(self.points)} retornos', ACCENT
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f'color: {color};')
        nearest = min((distance for distance, _ in self.points), default=None)
        self.nearest_label.setText(
            f'Mín.: {nearest:.2f} m' if self.has_fresh_points() else 'Mín.: —')
        ground_fresh = self._fresh(self._ground_received_at)
        self.ground_label.setText(
            f'Abaixo: {self.ground_distance:.2f} m'
            if ground_fresh and self.ground_distance is not None else 'Abaixo: —')
        self.canvas.update()
