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
    
    IMPORTANTE: Este enum deve estar sincronizado com o enum em fsm_node.py.
    """
    INICIAR_MISSAO = 1      # Iniciar missão (aceito apenas em PRONTO)
    CANCELAR_MISSAO = 2     # Cancelar missão e retornar (aceito em EXECUTANDO_*)


class DashboardFSMPublisher:
    """
    Gerencia a publicação de comandos de missão para o FSMNode.
    Usa DashboardFsmCommandMSG com código inteiro para type-safety.
    
    Comandos suportados:
    - INICIAR_MISSAO: Inicia uma missão (requer nome da missão de missions.json)
    - CANCELAR_MISSAO: Cancela a missão atual e inicia RTL
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        # QoS para comandos críticos: RELIABLE + TRANSIENT_LOCAL (garante entrega)
        # IMPORTANTE: Deve corresponder ao QoS do subscriber em fsm_node.py
        qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publisher para enviar comandos de missão para o fsm_node
        self.fsm_command_pub = self.DashboardNode.create_publisher(
            DashboardFsmCommandMSG,
            "/drone_inspetor/interno/dashboard_node/fsm_commands",
            qos_commands
        )
        
        self.DashboardNode.get_logger().info(f"Publicador para {self.fsm_command_pub.topic_name} criado.")

    def _send_command(self, command: DashboardFsmCommandDescription, mission: str = ""):
        """
        Método interno para enviar comandos para o FSM.
        
        Args:
            command: Código do comando (DashboardFsmCommandDescription)
            mission: Nome da missão (apenas para INICIAR_MISSAO, deve existir em missions.json)
        """
        msg = DashboardFsmCommandMSG()
        msg.command = int(command)
        msg.mission = mission
        self.fsm_command_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando enviado para FSM: {command.name}" + (f" (missão: {mission})" if mission else ""))

    def send_iniciar_missao(self, mission: str = "Flare"):
        """
        Envia comando para iniciar uma missão de inspeção.
        
        Args:
            mission: Nome da missão conforme definido em missions.json
        """
        self._send_command(DashboardFsmCommandDescription.INICIAR_MISSAO, mission)

    def send_cancelar_missao(self):
        """
        Envia comando para cancelar a missão atual.
        O drone irá executar RTL (Return To Launch) automaticamente.
        """
        self._send_command(DashboardFsmCommandDescription.CANCELAR_MISSAO)

    # Métodos de compatibilidade (deprecated - mantidos para transição)
    def send_mission_command(self, command_json: str):
        """
        DEPRECATED: Método de compatibilidade para comandos em formato JSON.
        Use send_iniciar_missao() ou send_cancelar_missao() diretamente.
        
        Args:
            command_json: String JSON com comando (formato legado)
        """
        import json
        try:
            cmd_dict = json.loads(command_json)
            cmd_name = cmd_dict.get("command", "").lower()
            
            if cmd_name in ["start_inspection", "iniciar_missao"]:
                mission = cmd_dict.get("inspection_type", cmd_dict.get("mission", "Flare"))
                self.send_iniciar_missao(mission)
            elif cmd_name in ["cancel_inspection", "cancelar_missao", "stop_inspection", 
                              "return_to_base", "abort_mission"]:
                self.send_cancelar_missao()
            else:
                self.DashboardNode.get_logger().warn(f"Comando não reconhecido: {cmd_name}")
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao parsear comando JSON: {e}")
