from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import json

class DashboardCameraPublisher:
    """
    Gerencia a publicação de comandos internos para controle da câmera.
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

        # Publisher para comandos de controle da câmera (tópico interno do dashboard)
        # Tópicos publicados pelo Dashboard para nodes: /drone_inspetor/dashboard/<topico>
        self.camera_control_pub = self.DashboardNode.create_publisher(
            String,
            "/drone_inspetor/interno/dashboard_node/camera/control",
            qos_commands
        )
        self.DashboardNode.get_logger().info(f"Publicador para {self.camera_control_pub.topic_name} criado.")

    def send_camera_control_command(self, command_dict):
        """
        Publica um comando de controle para a câmera.
        """
        msg = String()
        msg.data = json.dumps(command_dict)
        self.camera_control_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de controle da câmera {command_dict} publicado.")


