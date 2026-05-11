"""
Specs das ROS2 Actions do drone_inspetor.
"""

from drone_inspetor_msgs.action import DroneCommand

from drone_inspetor.ros_interfaces.specs import ActionSpec


class Actions:
    """Actions ROS2. Acessar via Topics.Action.<NOME>."""

    DRONE_COMMAND = ActionSpec(
        "/drone_inspetor/action/drone_command",
        DroneCommand,
    )
