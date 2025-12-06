from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import json

class DashboardLidarPublisher:
    """
    Gerencia a publicação de comandos internos para controle do LiDAR.
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

        # Publisher para comandos de controle do LiDAR (tópico interno do dashboard)
        # Tópicos publicados pelo Dashboard para nodes: /drone_inspetor/dashboard/<topico>
        self.lidar_control_pub = self.DashboardNode.create_publisher(
            String,
            "/drone_inspetor/dashboard/lidar/control",
            qos_commands
        )
        self.DashboardNode.get_logger().info(f"Publicador para {self.lidar_control_pub.topic_name} criado.")

    def send_lidar_control_command(self, command_dict):
        """
        Publica um comando de controle para o nó do LiDAR.
        """
        msg = String()
        msg.data = json.dumps(command_dict)
        self.lidar_control_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de controle do LiDAR {command_dict} publicado.")


