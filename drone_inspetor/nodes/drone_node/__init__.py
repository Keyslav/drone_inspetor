"""
drone_node package — interface exclusiva com PX4.

Contém:
- drone_node.py: entry point + classe DroneNode (herda de mixins)
- px4_state.py: DroneStatePX4 (container de telemetria PX4)
- obstacles.py: DroneObstacles (detecção e evitação)
- state_machine.py: DroneState (FSM interna do drone)
- trajectory.py: DroneTrajectoryMixin (cálculo de setpoints e GPS)
- action_server.py: DroneActionServerMixin (callbacks DroneCommand)
- px4_commands.py: DronePX4CommandsMixin (arm, takeoff, goto, rtl, land, stop)
"""

from drone_inspetor.nodes.drone_node.drone_node import DroneNode, main

__all__ = ["DroneNode", "main"]
