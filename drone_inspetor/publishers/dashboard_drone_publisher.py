from rclpy.node import Node
from std_msgs.msg import String
from drone_inspetor.ros_interfaces import Topics, create_publisher_from
import json

class DashboardDronePublisher:
    """
    Gerencia a publicação de comandos de controle e missão.
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        self.drone_command_pub = create_publisher_from(self.DashboardNode, Topics.Dashboard.DRONE_COMMANDS)
        self.DashboardNode.get_logger().info(f"Publicador para {self.drone_command_pub.topic_name} criado.")

    def send_mission_command(self, command_str):
        """
        Publica um comando de missão no tópico ROS2.
        """
        msg = String()
        msg.data = command_str
        self.drone_command_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de missão para Drone\'{command_str}\' publicado.")

