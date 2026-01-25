from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from drone_inspetor_msgs.msg import CVControlMSG
import json

class DashboardCVPublisher:
    """
    Gerencia a publicação de comandos internos para controle de Visão Computacional.
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

        # Publisher para comandos de controle de CV (tópico interno do dashboard) - legado
        self.cv_control_pub = self.DashboardNode.create_publisher(
            String,
            "/drone_inspetor/dashboard/cv/control",
            qos_commands
        )
        self.DashboardNode.get_logger().info(f"Publicador para {self.cv_control_pub.topic_name} criado.")
        
        # Publisher para controle de modelos CV (nova mensagem CVControlMSG)
        self.cv_model_control_pub = self.DashboardNode.create_publisher(
            CVControlMSG,
            "/drone_inspetor/interno/dashboard_node/cv_node/cv_control",
            qos_commands
        )
        self.DashboardNode.get_logger().info(f"Publisher CVControlMSG criado: {self.cv_model_control_pub.topic_name}")

    def send_cv_control_command(self, command_dict):
        """
        Publica um comando de controle para o nó de Visão Computacional (legado).
        """
        msg = String()
        msg.data = json.dumps(command_dict)
        self.cv_control_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de controle CV {command_dict} publicado.")
    
    def publish_cv_control(self, object_model: str, anomaly_model: str):
        """
        Publica mensagem de controle para seleção de modelos CV.
        
        Args:
            object_model (str): Nome do arquivo do modelo de detecção de objetos
            anomaly_model (str): Nome do arquivo do modelo de detecção de anomalias
        """
        msg = CVControlMSG()
        msg.object_detection_model = object_model
        msg.anomaly_detection_model = anomaly_model
        self.cv_model_control_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"CVControlMSG: obj={object_model}, anom={anomaly_model}")
