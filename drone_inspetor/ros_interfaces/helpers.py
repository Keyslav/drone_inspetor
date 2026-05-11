"""
Helpers de criação de pub/sub/service/action a partir de Specs.

Eliminam call-sites com 4 parâmetros independentes (msg_type, name, callback, qos)
em favor de um único spec que carrega o contrato completo.
"""

from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from drone_inspetor.ros_interfaces.specs import ActionSpec, ServiceSpec, TopicSpec


def create_subscription_from(node: Node, spec: TopicSpec, callback, callback_group=None):
    """Cria um subscriber usando o contrato completo do TopicSpec."""
    return node.create_subscription(
        spec.msg_type,
        spec.name,
        callback,
        spec.qos,
        callback_group=callback_group,
    )


def create_publisher_from(node: Node, spec: TopicSpec):
    """Cria um publisher usando o contrato completo do TopicSpec."""
    return node.create_publisher(spec.msg_type, spec.name, spec.qos)


def create_service_from(node: Node, spec: ServiceSpec, callback, callback_group=None):
    """Cria um service server usando o contrato do ServiceSpec."""
    return node.create_service(
        spec.srv_type,
        spec.name,
        callback,
        callback_group=callback_group,
    )


def create_client_from(node: Node, spec: ServiceSpec, callback_group=None):
    """Cria um service client usando o contrato do ServiceSpec."""
    return node.create_client(spec.srv_type, spec.name, callback_group=callback_group)


def make_action_server(
    node: Node,
    spec: ActionSpec,
    execute_callback,
    goal_callback=None,
    cancel_callback=None,
    callback_group=None,
):
    """Cria um ActionServer usando o contrato do ActionSpec."""
    kwargs = {"execute_callback": execute_callback}
    if goal_callback is not None:
        kwargs["goal_callback"] = goal_callback
    if cancel_callback is not None:
        kwargs["cancel_callback"] = cancel_callback
    if callback_group is not None:
        kwargs["callback_group"] = callback_group
    return ActionServer(node, spec.action_type, spec.name, **kwargs)


def make_action_client(node: Node, spec: ActionSpec, callback_group=None):
    """Cria um ActionClient usando o contrato do ActionSpec."""
    if callback_group is not None:
        return ActionClient(node, spec.action_type, spec.name, callback_group=callback_group)
    return ActionClient(node, spec.action_type, spec.name)
