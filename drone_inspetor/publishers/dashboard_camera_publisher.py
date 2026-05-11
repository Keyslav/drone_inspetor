from rclpy.node import Node
from std_msgs.msg import String
from drone_inspetor.ros_interfaces import Topics, create_publisher_from
import json

class DashboardCameraPublisher:
    """
    Gerencia a publicação de comandos internos para controle da câmera.
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        self.camera_control_pub = create_publisher_from(self.DashboardNode, Topics.Dashboard.CAMERA_CONTROL)
        self.DashboardNode.get_logger().info(f"Publicador para {self.camera_control_pub.topic_name} criado.")

    def send_camera_control_command(self, command_dict):
        """
        Publica um comando de controle para a câmera.
        """
        msg = String()
        msg.data = json.dumps(command_dict)
        self.camera_control_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de controle da câmera {command_dict} publicado.")


