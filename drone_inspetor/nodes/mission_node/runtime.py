"""Dependências explícitas da FSM, desacopladas da classe rclpy.Node."""

import time
from dataclasses import dataclass
from typing import Callable

from drone_inspetor.nodes.mission_node.action_client import DroneActionClient
from drone_inspetor.nodes.mission_node.config import MissionConfig
from drone_inspetor.nodes.mission_node.cv_client import CVClient


@dataclass
class MissionRuntime:
    """Fornece somente telemetria, operações, relógio ROS e diagnóstico aos estados."""

    drone: object
    actions: DroneActionClient
    cv: CVClient
    config: MissionConfig
    logger: object
    ros_time: Callable[[], float]
    telemetry_healthy: Callable[[], bool]
    monotonic_time: Callable[[], float] = time.monotonic

    def get_logger(self):
        """Mantém a interface de log dos estados sem fornecer o nó inteiro."""
        return self.logger

    def now(self):
        """Tempo ROS: permanência pausa junto da simulação."""
        return self.ros_time()
