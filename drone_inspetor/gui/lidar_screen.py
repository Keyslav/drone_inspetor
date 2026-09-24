"""Adaptador dos sinais do dashboard para o radar Qt, sem dependência de navegador."""

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

from .widgets.lidar_radar import LidarRadar


class LidarScreen(QWidget):
    """Preserva os callbacks públicos; ROS entrega apenas tipos nativos à GUI."""

    point_vector_received = pyqtSignal(object)
    statistics_received = pyqtSignal(object)
    obstacle_detections_received = pyqtSignal(object)

    def __init__(self, signals, original_label: QLabel):
        super().__init__()
        self.original_label = original_label
        self.radar = LidarRadar(original_label)
        layout = original_label.layout() or QVBoxLayout(original_label)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.radar)
        original_label.setText('')

    def update_point_vector(self, point_vector: list):
        """Vetor alterna distância em metros e ângulo em radianos no frame FLU."""
        self.radar.set_points(point_vector)

    def update_ground_distance(self, distance: float):
        """Distância ao obstáculo abaixo, distinta da altitude no referencial PX4."""
        self.radar.set_ground_distance(distance)

    def update_lidar_statistics(self, statistics: dict):
        """Compatibilidade: estatísticas visuais são derivadas do próprio scan."""

    def update_obstacle_detections(self, detections: dict):
        """Flags legadas não substituem distâncias ou certificam espaço livre."""
