from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import IntEnum

from drone_inspetor_msgs.msg import DashboardFsmCommandMSG


# ==================================================================================================
# ENUM DE COMANDOS DO DASHBOARD PARA FSM
# ==================================================================================================

class DashboardFsmCommandDescription(IntEnum):
    """
    Enum que representa os comandos enviados do Dashboard para o FSM Node.
    Os valores inteiros correspondem ao campo 'command' de DashboardFsmCommandMSG.
    """
    START_INSPECTION = 1           # Iniciar missão de inspeção
    STOP_INSPECTION = 2            # Pausar missão (drone para no ar)
    CANCEL_INSPECTION = 3          # Cancelar missão e retornar à base
    RETURN_TO_BASE = 4             # Retornar à base (RTL)
    ENABLE_OFFBOARD_CONTROL_MODE = 5  # Habilitar modo offboard (compatibilidade)
    ABORT_MISSION = 6              # Abortar missão e retornar à base
    EMERGENCY_LAND = 7             # Pouso de emergência imediato


class DashboardFSMPublisher:
    """
    Gerencia a publicação de comandos de missão para o FSMNode.
    Usa DashboardFsmCommandMSG com código inteiro para type-safety.
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
            DashboardFsmCommandMSG,
            "/drone_inspetor/interno/dashboard_node/fsm_commands",
            qos_commands
        )
        
        self.DashboardNode.get_logger().info(f"Publicador para {self.fsm_command_pub.topic_name} criado.")

    def _send_command(self, command: DashboardFsmCommandDescription, inspection_type: str = ""):
        """
        Método interno para enviar comandos para o FSM.
        
        Args:
            command: Código do comando (DashboardFsmCommandDescription)
            inspection_type: Tipo de inspeção (apenas para START_INSPECTION)
        """
        msg = DashboardFsmCommandMSG()
        msg.command = int(command)
        msg.inspection_type = inspection_type
        self.fsm_command_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando enviado para FSM: {command.name}")

    def send_start_inspection(self, inspection_type: str = "Flare"):
        """Envia comando para iniciar missão de inspeção."""
        self._send_command(DashboardFsmCommandDescription.START_INSPECTION, inspection_type)

    def send_stop_inspection(self):
        """Envia comando para pausar missão (drone para no ar)."""
        self._send_command(DashboardFsmCommandDescription.STOP_INSPECTION)

    def send_cancel_inspection(self):
        """Envia comando para cancelar missão e retornar à base."""
        self._send_command(DashboardFsmCommandDescription.CANCEL_INSPECTION)

    def send_return_to_base(self):
        """Envia comando para retornar à base (RTL)."""
        self._send_command(DashboardFsmCommandDescription.RETURN_TO_BASE)

    def send_enable_offboard_control_mode(self):
        """Envia comando para habilitar modo offboard (compatibilidade)."""
        self._send_command(DashboardFsmCommandDescription.ENABLE_OFFBOARD_CONTROL_MODE)

    def send_abort_mission(self):
        """Envia comando para abortar missão e retornar à base."""
        self._send_command(DashboardFsmCommandDescription.ABORT_MISSION)

    def send_emergency_land(self):
        """Envia comando para pouso de emergência imediato."""
        self._send_command(DashboardFsmCommandDescription.EMERGENCY_LAND)
