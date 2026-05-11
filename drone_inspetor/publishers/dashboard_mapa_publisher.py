from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from drone_inspetor.ros_interfaces import Topics, create_publisher_from
import json

class DashboardMapaPublisher:
    """
    Gerencia a publicação de comandos internos para o Mapa.
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        self.mapa_position_pub = create_publisher_from(self.DashboardNode, Topics.Dashboard.MAPA_POSITION_CMD)
        self.DashboardNode.get_logger().info(f"Publicador para {self.mapa_position_pub.topic_name} criado.")

        self.mapa_attitude_pub = create_publisher_from(self.DashboardNode, Topics.Dashboard.MAPA_ATTITUDE_CMD)
        self.DashboardNode.get_logger().info(f"Publicador para {self.mapa_attitude_pub.topic_name} criado.")

    def send_mapa_position_command(self, lat, lon, alt):
        """
        Publica um comando de posição para o mapa.
        """
        msg = PoseStamped()
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = lon
        msg.pose.position.altitude = alt
        self.mapa_position_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de posição do mapa lat:{lat}, lon:{lon}, alt:{alt} publicado.")

    def send_mapa_attitude_command(self, roll, pitch, yaw):
        """
        Publica um comando de atitude para o mapa.
        """
        msg = String()
        data = {"roll": roll, "pitch": pitch, "yaw": yaw}
        msg.data = json.dumps(data)
        self.mapa_attitude_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de atitude do mapa {data} publicado.")


