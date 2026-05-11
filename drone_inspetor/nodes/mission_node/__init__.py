"""
mission_node package — máquina de estados de missão hierárquica.

Contém:
- mission_node.py: entry point + classe MissionNode (nó ROS2)
- state_machine.py: MissionStateMachine (FSM de missão com match/case)
- drone_state_data.py: DroneStateData (espelho de telemetria do drone)
"""

from drone_inspetor.nodes.mission_node.mission_node import MissionNode, main

__all__ = ["MissionNode", "main"]
