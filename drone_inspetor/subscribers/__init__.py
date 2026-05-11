"""
Subscribers do dashboard_node — recebem dados dos demais nós e emitem sinais PyQt.

Cada subscriber assina um tópico específico, converte mensagens ROS para tipos
nativos (via msg_to_dict ou conversão manual) e emite sinais PyQt6.
"""

from drone_inspetor.subscribers.dashboard_camera_subscriber import DashboardCameraSubscriber
from drone_inspetor.subscribers.dashboard_cv_subscriber import DashboardCVSubscriber
from drone_inspetor.subscribers.dashboard_depth_subscriber import DashboardDepthSubscriber
from drone_inspetor.subscribers.dashboard_drone_subscriber import DashboardDroneSubscriber
from drone_inspetor.subscribers.dashboard_lidar_subscriber import DashboardLidarSubscriber
from drone_inspetor.subscribers.dashboard_mapa_subscriber import DashboardMapaSubscriber
from drone_inspetor.subscribers.dashboard_mission_subscriber import DashboardMissionSubscriber

__all__ = [
    "DashboardCameraSubscriber",
    "DashboardCVSubscriber",
    "DashboardDepthSubscriber",
    "DashboardDroneSubscriber",
    "DashboardLidarSubscriber",
    "DashboardMapaSubscriber",
    "DashboardMissionSubscriber",
]
