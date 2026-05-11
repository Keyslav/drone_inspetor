"""
Sinais PyQt6 para comunicação assíncrona ROS2 ↔ GUI.

DashboardSignals agrega sinais modulares por componente:
- camera, cv, depth, lidar (sensores)
- mission, control (estados/comandos)
- mapa (GPS/visualização)
"""

from drone_inspetor.signals.dashboard_signals import (
    DashboardSignals,
    CameraSignals,
    CVSignals,
    DepthSignals,
    LidarSignals,
    MissionSignals,
    DroneSignals,
    MapaSignals,
)

__all__ = [
    "DashboardSignals",
    "CameraSignals",
    "CVSignals",
    "DepthSignals",
    "LidarSignals",
    "MissionSignals",
    "DroneSignals",
    "MapaSignals",
]
