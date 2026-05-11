"""
Dataclasses que representam o contrato completo de cada canal de comunicação ROS2.

Cada spec amarra nome + tipo (e QoS quando aplicável) em um único objeto imutável,
eliminando a possibilidade de pub/sub usarem QoS ou tipos divergentes.
"""

from dataclasses import dataclass

from rclpy.qos import QoSProfile


@dataclass(frozen=True)
class TopicSpec:
    """Contrato de um tópico pub/sub: nome, tipo de mensagem e perfil QoS."""

    name: str
    msg_type: type
    qos: QoSProfile


@dataclass(frozen=True)
class ServiceSpec:
    """Contrato de um serviço: nome e tipo. ROS2 define QoS de serviços internamente."""

    name: str
    srv_type: type


@dataclass(frozen=True)
class ActionSpec:
    """Contrato de uma action: nome e tipo. ROS2 define QoS de actions internamente."""

    name: str
    action_type: type
