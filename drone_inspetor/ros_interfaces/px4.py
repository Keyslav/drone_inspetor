"""
Specs dos tópicos do firmware PX4 (`/fmu/in/*` e `/fmu/out/*`).

Todos usam QoSProfiles.px4() — perfil compatível com o middleware uXRCE-DDS.
Esses tópicos são consumidos exclusivamente pelo drone_node.
"""

from px4_msgs.msg import (
    BatteryStatus,
    HomePosition,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleCommandAck,
    VehicleGlobalPosition,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from drone_inspetor.ros_interfaces.qos import QoSProfiles
from drone_inspetor.ros_interfaces.specs import TopicSpec


_PX4_QOS = QoSProfiles.px4()


class PX4Topics:
    """Tópicos PX4. Acessar via Topics.PX4.<NOME>."""

    # --- Telemetria (/fmu/out/*) ---
    VEHICLE_STATUS = TopicSpec("/fmu/out/vehicle_status_v1", VehicleStatus, _PX4_QOS)
    VEHICLE_COMMAND_ACK = TopicSpec("/fmu/out/vehicle_command_ack", VehicleCommandAck, _PX4_QOS)
    VEHICLE_LOCAL_POSITION = TopicSpec("/fmu/out/vehicle_local_position", VehicleLocalPosition, _PX4_QOS)
    VEHICLE_GLOBAL_POSITION = TopicSpec("/fmu/out/vehicle_global_position", VehicleGlobalPosition, _PX4_QOS)
    HOME_POSITION = TopicSpec("/fmu/out/home_position", HomePosition, _PX4_QOS)
    VEHICLE_ATTITUDE = TopicSpec("/fmu/out/vehicle_attitude", VehicleAttitude, _PX4_QOS)
    VEHICLE_LAND_DETECTED = TopicSpec("/fmu/out/vehicle_land_detected", VehicleLandDetected, _PX4_QOS)
    BATTERY_STATUS = TopicSpec("/fmu/out/battery_status", BatteryStatus, _PX4_QOS)

    # --- Comandos (/fmu/in/*) ---
    VEHICLE_COMMAND = TopicSpec("/fmu/in/vehicle_command", VehicleCommand, _PX4_QOS)
    OFFBOARD_CONTROL_MODE = TopicSpec("/fmu/in/offboard_control_mode", OffboardControlMode, _PX4_QOS)
    TRAJECTORY_SETPOINT = TopicSpec("/fmu/in/trajectory_setpoint", TrajectorySetpoint, _PX4_QOS)
