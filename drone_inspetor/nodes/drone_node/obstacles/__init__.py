"""
obstacles/ — subsistema de detecção de obstáculos do DroneNode.

Estrutura:
    drone_obstacle.py       → DroneObstacle: armazena flags agregadas (OR entre fontes).
    lidar_obstacles/        → LidarObstacle: processa LidarMSG (dados brutos) → flags.
    depth_obstacles/        → DepthObstacle: recebe ObstaclesMSG → flags frontais.
"""

from drone_inspetor.nodes.drone_node.obstacles.drone_obstacle import DroneObstacle

__all__ = ["DroneObstacle"]
