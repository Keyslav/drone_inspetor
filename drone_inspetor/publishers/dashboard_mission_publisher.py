from rclpy.node import Node

from drone_inspetor_msgs.msg import DashboardMissionCommandMSG
from drone_inspetor.common.enums import DashboardMissionCommandDescription
from drone_inspetor.ros_interfaces import Topics, create_publisher_from


class DashboardMissionPublisher:
    """
    Gerencia a publicação de comandos de missão para o MissionNode.
    Usa DashboardMissionCommandMSG com código inteiro para type-safety.

    Comandos suportados:
    - INICIAR_MISSAO: Inicia uma missão (requer nome da missão de missions.json)
    - CANCELAR_MISSAO: Cancela a missão atual e inicia RTL
    """
    def __init__(self, DashboardNode: Node):
        self.DashboardNode = DashboardNode

        # Publisher para enviar comandos de missão para o mission_node
        self.mission_command_pub = create_publisher_from(self.DashboardNode, Topics.Dashboard.MISSION_COMMANDS)

        self.DashboardNode.get_logger().info(f"Publicador para {self.mission_command_pub.topic_name} criado.")

    def _send_command(self, command: DashboardMissionCommandDescription, mission: str = ""):
        """
        Método interno para enviar comandos para o Mission Node.

        Args:
            command: Código do comando (DashboardMissionCommandDescription)
            mission: Nome da missão (apenas para INICIAR_MISSAO, deve existir em missions.json)
        """
        msg = DashboardMissionCommandMSG()
        msg.command = int(command)
        msg.mission = mission
        self.mission_command_pub.publish(msg)
        self.DashboardNode.get_logger().info(f"Comando enviado para Mission Node: {command.name}" + (f" (missão: {mission})" if mission else ""))

    def send_iniciar_missao(self, mission: str = "Flare"):
        """
        Envia comando para iniciar uma missão de inspeção.

        Args:
            mission: Nome da missão conforme definido em missions.json
        """
        self._send_command(DashboardMissionCommandDescription.INICIAR_MISSAO, mission)

    def send_cancelar_missao(self):
        """
        Envia comando para cancelar a missão atual.
        O drone irá executar RTL (Return To Launch) automaticamente.
        """
        self._send_command(DashboardMissionCommandDescription.CANCELAR_MISSAO)

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
