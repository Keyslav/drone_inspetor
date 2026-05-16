"""
drone_node package — interface exclusiva com PX4.

Contém:
- drone_node.py: entry point + classe DroneNode (herda de mixins)
- px4_state.py: DroneStatePX4 (container de telemetria PX4)
- obstacles/: subsistema de detecção de obstáculos
    - drone_obstacle.py: DroneObstacle (armazenamento central de flags, OR entre fontes)
    - lidar_obstacles/: LidarObstacle (processa LidarMSG → flags)
    - depth_obstacles/: DepthObstacle (recebe ObstaclesMSG → flags frontais)
- obstacle_avoidance.py: helpers de evasão e consulta (has_obstacle_in_sector, velocity cap, etc.)
- fsm/: máquinas de estado (DroneFSM + DeslocamentoFSM)
- trajectory.py: DroneTrajectoryMixin (cálculo de setpoints e GPS)
- action_server.py: DroneActionServerMixin (callbacks DroneCommand)
- px4_commands.py: DronePX4CommandsMixin (arm, takeoff, goto, rtl, land, stop)
"""

from drone_inspetor.nodes.drone_node.drone_node import DroneNode, main

__all__ = ["DroneNode", "main"]
