from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json

class DashboardMapaPublisher:
    """
    Gerencia a publicação de comandos internos para o Mapa.
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

        # Publisher para comandos de atualização de posição no mapa (tópico interno do dashboard)
        self.mapa_position_pub = self.DashboardNode.create_publisher(
            PoseStamped,
            "/drone_inspetor/dashboard/mapa/position_command",
            qos_commands
        )
        self.DashboardNode.get_logger().info(f"Publicador para {self.mapa_position_pub.topic_name} criado.")

        # Publisher para comandos de atualização de atitude no mapa (tópico interno do dashboard)
        self.mapa_attitude_pub = self.DashboardNode.create_publisher(
            String,
            "/drone_inspetor/dashboard/mapa/attitude_command",
            qos_commands
        )
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


