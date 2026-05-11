from rclpy.node import Node
from std_msgs.msg import String
from drone_inspetor.ros_interfaces import Topics, create_publisher_from
import json

class DashboardLidarPublisher:
    """
    Gerencia a publicação de comandos internos para controle do LiDAR.
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        self.lidar_control_pub = create_publisher_from(self.DashboardNode, Topics.Dashboard.LIDAR_CONTROL)
        self.DashboardNode.get_logger().info(f"Publicador para {self.lidar_control_pub.topic_name} criado.")

    def send_lidar_control_command(self, command_dict):
        """
        Publica um comando de controle para o nó do LiDAR.
        """
        msg = String()
        msg.data = json.dumps(command_dict)
        self.lidar_control_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de controle do LiDAR {command_dict} publicado.")


