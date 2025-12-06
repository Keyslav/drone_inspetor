from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import json

class DashboardDepthPublisher:
    """
    Gerencia a publicação de comandos internos para controle de profundidade.
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        # QoS para comandos simples: VOLATILE + RELIABLE (garantir entrega de comandos)
        qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher para comandos de controle de profundidade (tópico interno do dashboard)
        self.depth_control_pub = self.DashboardNode.create_publisher(
            String,
            "/drone_inspetor/dashboard/depth/control",
            qos_commands
        )
        self.DashboardNode.get_logger().info(f"Publicador para {self.depth_control_pub.topic_name} criado.")

    def send_depth_control_command(self, command_dict):
        """
        Publica um comando de controle para o nó de Profundidade.
        """
        msg = String()
        msg.data = json.dumps(command_dict)
        self.depth_control_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de controle de profundidade {command_dict} publicado.")


