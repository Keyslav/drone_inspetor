"""
Pacote `ros_interfaces` — fonte única de verdade dos canais de comunicação ROS2 do drone_inspetor.

Cada tópico/serviço/action é representado por uma spec que amarra nome + tipo (+ QoS),
eliminando a possibilidade de pub/sub divergirem em runtime.

Uso típico:

    from drone_inspetor.ros_interfaces import Topics, create_subscription_from, create_publisher_from

    create_subscription_from(self, Topics.Interno.DRONE_STATE, self.callback)
    create_publisher_from(self, Topics.PX4.VEHICLE_COMMAND)
"""

from drone_inspetor.ros_interfaces.actions import Actions
from drone_inspetor.ros_interfaces.dashboard import DashboardTopics
from drone_inspetor.ros_interfaces.external import ExternalTopics
from drone_inspetor.ros_interfaces.helpers import (
    create_client_from,
    create_publisher_from,
    create_service_from,
    create_subscription_from,
    make_action_client,
    make_action_server,
)
from drone_inspetor.ros_interfaces.internal import InternalTopics
from drone_inspetor.ros_interfaces.px4 import PX4Topics
from drone_inspetor.ros_interfaces.qos import QoSProfiles
from drone_inspetor.ros_interfaces.services import Services
from drone_inspetor.ros_interfaces.specs import ActionSpec, ServiceSpec, TopicSpec


class Topics:
    """Namespace raiz de todos os canais de comunicação ROS2."""

    PX4 = PX4Topics
    Interno = InternalTopics
    Externo = ExternalTopics
    Dashboard = DashboardTopics
    Service = Services
    Action = Actions


__all__ = [
    "Topics",
    "TopicSpec",
    "ServiceSpec",
    "ActionSpec",
    "QoSProfiles",
    "create_subscription_from",
    "create_publisher_from",
    "create_service_from",
    "create_client_from",
    "make_action_server",
    "make_action_client",
]
