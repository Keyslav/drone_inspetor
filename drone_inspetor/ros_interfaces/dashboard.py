"""
Specs dos tópicos do Dashboard (/drone_inspetor/interno/dashboard_node/* e /drone_inspetor/dashboard/*).

Comandos vindos da GUI PyQt6 para os nós do sistema.
"""

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from drone_inspetor_msgs.msg import CVControlMSG, DashboardMissionCommandMSG

from drone_inspetor.ros_interfaces.qos import QoSProfiles
from drone_inspetor.ros_interfaces.specs import TopicSpec


class DashboardTopics:
    """Tópicos do dashboard. Acessar via Topics.Dashboard.<NOME>."""

    # --- Comandos críticos (RELIABLE + TRANSIENT_LOCAL) ---
    MISSION_COMMANDS = TopicSpec(
        "/drone_inspetor/interno/dashboard_node/mission_commands",
        DashboardMissionCommandMSG,
        QoSProfiles.commands(),
    )

    # --- Comandos de controle (RELIABLE + VOLATILE) ---
    DRONE_COMMANDS = TopicSpec(
        "/drone_inspetor/interno/dashboard_node/drone_commands",
        String,
        QoSProfiles.commands_volatile(),
    )
    CAMERA_CONTROL = TopicSpec(
        "/drone_inspetor/interno/dashboard_node/camera/control",
        String,
        QoSProfiles.commands_volatile(),
    )
    CV_CONTROL = TopicSpec(
        "/drone_inspetor/interno/dashboard_node/cv_node/cv_control",
        CVControlMSG,
        QoSProfiles.commands_volatile(),
    )
    CV_CONTROL_LEGACY = TopicSpec(
        "/drone_inspetor/dashboard/cv/control",
        String,
        QoSProfiles.commands_volatile(),
    )
    DEPTH_CONTROL = TopicSpec(
        "/drone_inspetor/dashboard/depth/control",
        String,
        QoSProfiles.commands_volatile(),
    )
    LIDAR_CONTROL = TopicSpec(
        "/drone_inspetor/dashboard/lidar/control",
        String,
        QoSProfiles.commands_volatile(),
    )

    # --- Mapa GPS interativo ---
    MAPA_POSITION = TopicSpec(
        "/drone_inspetor/dashboard/mapa/position",
        PoseStamped,
        QoSProfiles.sensor_data(),
    )
    MAPA_ATTITUDE = TopicSpec(
        "/drone_inspetor/dashboard/mapa/attitude",
        String,
        QoSProfiles.sensor_data(),
    )
    MAPA_POSITION_CMD = TopicSpec(
        "/drone_inspetor/dashboard/mapa/position_command",
        PoseStamped,
        QoSProfiles.commands_volatile(),
    )
    MAPA_ATTITUDE_CMD = TopicSpec(
        "/drone_inspetor/dashboard/mapa/attitude_command",
        String,
        QoSProfiles.commands_volatile(),
    )
