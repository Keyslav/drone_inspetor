from rclpy.node import Node
from std_msgs.msg import String
from drone_inspetor.ros_interfaces import Topics, create_publisher_from
import json

class DashboardDepthPublisher:
    """
    Gerencia a publicação de comandos internos para controle de profundidade.
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        self.depth_control_pub = create_publisher_from(self.DashboardNode, Topics.Dashboard.DEPTH_CONTROL)
        self.DashboardNode.get_logger().info(f"Publicador para {self.depth_control_pub.topic_name} criado.")

    def send_depth_control_command(self, command_dict):
        """
        Publica um comando de controle para o nó de Profundidade.
        """
        msg = String()
        msg.data = json.dumps(command_dict)
        self.depth_control_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de controle de profundidade {command_dict} publicado.")


