from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import json

class DashboardDronePublisher:
    """
    Gerencia a publicação de comandos de controle e missão.
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

        # Publisher para enviar comandos de missão para o drone_node
        self.control_command_pub = self.DashboardNode.create_publisher(
            String,
            "/drone_inspetor/interno/dashboard_node/control_commands",
            qos_commands
        )
        self.DashboardNode.get_logger().info(f"Publicador para {self.control_command_pub.topic_name} criado.")

    def send_mission_command(self, command_str):
        """
        Publica um comando de missão no tópico ROS2.
        """
        msg = String()
        msg.data = command_str
        self.control_command_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de missão para Controle\'{command_str}\' publicado.")

