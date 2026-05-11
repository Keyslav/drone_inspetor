"""Módulo compartilhado com constantes e enums do drone_inspetor.

Tópicos e perfis QoS foram movidos para `drone_inspetor.ros_interfaces`.
"""

from drone_inspetor.common.enums import (
    DroneStateDescription,
    MissionStateDescription,
    DashboardMissionCommandDescription,
    DRONE_STATES_GOTO,
    DRONE_STATES_GOTO_COM_FOCO,
    DRONE_STATES_RTL,
    DRONE_STATES_POUSANDO,
    DRONE_STATES_POUSADO,
    DRONE_STATES_EM_MOVIMENTO,
)
from drone_inspetor.common.constants import DroneConstants
