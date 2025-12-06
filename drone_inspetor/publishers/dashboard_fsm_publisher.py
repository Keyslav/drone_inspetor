from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

class DashboardFSMPublisher:
    """
    Gerencia a publicação de comandos de missão para o FSMNode.
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        # QoS para comandos críticos: TRANSIENT_LOCAL + BEST_EFFORT (baixa latência, disponível para novos subscribers)
        qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher para enviar comandos de missão para o fsm_node
        self.fsm_command_pub = self.DashboardNode.create_publisher(
            String,
            "/drone_inspetor/interno/dashboard_node/mission_commands",
            qos_commands
        )
        
        self.DashboardNode.get_logger().info(f"Publicador para {self.fsm_command_pub.topic_name} criado.")

    def send_mission_command(self, command_json: str):
        """
        Publica um comando de missão para o FSMNode.
        
        Formato esperado: JSON string contendo:
        {
            "command": "<nome_do_comando>",
            ... outras variáveis específicas do comando ...
        }
        
        Args:
            command_json (str): String JSON contendo o comando de missão e suas variáveis
        """
        msg = String()
        msg.data = command_json
        self.fsm_command_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando de missão enviado para FSM (JSON): {command_json}")


