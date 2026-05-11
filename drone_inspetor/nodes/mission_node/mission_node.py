# mission_node.py
# =================================================================================================
# NÓ DA MÁQUINA DE ESTADOS DE MISSÃO (O "CÉREBRO")
# =================================================================================================
# RESPONSABILIDADE PRINCIPAL:
# Gerenciar o fluxo lógico e sequencial da missão de inspeção. Implementa uma máquina de estados hierárquica
# com estados e sub-estados para controlar o comportamento do drone durante
# toda a missão.
#
# =================================================================================================

import rclpy
from rclpy.node import Node
from rclpy.action.client import ClientGoalHandle
import time
import math

# Type hints / construtores de mensagens customizadas
from drone_inspetor_msgs.msg import (
    CVDetectionMSG,
    DashboardMissionCommandMSG,
    DroneStateMSG,
)
from drone_inspetor_msgs.action import DroneCommand

# Importações centralizadas (fonte única de verdade)
from drone_inspetor.common.enums import (
    DroneStateDescription,
    MissionStateDescription,
    DashboardMissionCommandDescription,
    DRONE_STATES_GOTO,
    DRONE_STATES_GOTO_COM_FOCO,
    DRONE_STATES_RTL,
    DRONE_STATES_POUSANDO,
    DRONE_STATES_POUSADO,
    DRONE_STATES_EM_MOVIMENTO,
)
from drone_inspetor.common.log_colors import LogPrefix
from drone_inspetor.ros_interfaces import (
    Topics,
    create_client_from,
    create_publisher_from,
    create_subscription_from,
    make_action_client,
)


# Importações internas do mission_node
from drone_inspetor.nodes.mission_node.drone_state_data import DroneStateData
from drone_inspetor.nodes.mission_node.fsm.context import MissionFSMContext
from drone_inspetor.nodes.mission_node.fsm.machine import MissionFSMachine


# ==================================================================================================
# CLASSE MissionNode
# ==================================================================================================

class MissionNode(Node):
    """
    Implementa a lógica de estados hierárquica da missão de inspeção.
    
    FLUXO DA MISSÃO:
    1. DESATIVADO: Falha em algum dos tópicos essenciais
    2. PRONTO: Drone pronto e aguardando comando de missão do dashboard
    3. EXECUTANDO_ARMANDO: Arma os motores
    4. EXECUTANDO_DECOLANDO: Decola para altitude de missão
    5. EXECUTANDO_INSPECIONANDO: Percorre pontos de inspeção da missão
    6. INSPECAO_FINALIZADA: Transição para retorno
    7. RETORNANDO: RTL, pousa e desarma
    """



    # Mapeamento de comandos do dashboard para estados permitidos
    VALID_DASHBOARD_COMMANDS = {
        DashboardMissionCommandDescription.INICIAR_MISSAO: [
            MissionStateDescription.PRONTO,
        ],
        DashboardMissionCommandDescription.CANCELAR_MISSAO: [
            MissionStateDescription.EXECUTANDO_ARMANDO,
            MissionStateDescription.EXECUTANDO_DECOLANDO,
            MissionStateDescription.EXECUTANDO_INSPECIONANDO,
            MissionStateDescription.EXECUTANDO_INSPECIONANDO_DETECTANDO,
            MissionStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEANDO,
            MissionStateDescription.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO,
            MissionStateDescription.EXECUTANDO_INSPECIONANDO_FALHA,
            MissionStateDescription.INSPECAO_FINALIZADA,
        ],
    }
    
    def __init__(self):
        # --- Inicialização do Nó ROS2 ---
        super().__init__("mission_node")
        self.get_logger().info("================ INICIALIZANDO MISSION NODE ===========")

        # --- Instâncias das Classes de Estado ---
        self.drone = DroneStateData()
        self.mission_ctx = MissionFSMContext(self)
        self.mission_machine = MissionFSMachine(self.mission_ctx, self)
        self.mission_machine.register_all_states()
        self.mission_machine.transition_to(MissionStateDescription.DESATIVADO)

        # --- Verificação de Saúde dos Tópicos Essenciais ---
        self.essencial_topics = {
            "drone_state": {"last_received": 0.0, "description": "Estado do drone_node"},
        }
        self.topic_health_timeout = 5.0

        # ==================================================================
        # PUBLISHERS
        # ==================================================================
        # Publica o estado atual da máquina de missão para o dashboard
        self.mission_state_pub = create_publisher_from(self, Topics.Interno.MISSION_STATE)

        # ==================================================================
        # ACTION CLIENT (comunicação com drone_node)
        # ==================================================================
        # Substitui o publisher de tópico por ActionClient para:
        # - Feedback durante execução de comandos
        # - Resultado final da ação
        # - Capacidade de cancelar ações em andamento
        self._action_client = make_action_client(self, Topics.Action.DRONE_COMMAND)

        # Variáveis para controle de action em andamento
        self._current_goal_handle: ClientGoalHandle = None
        self._last_action_feedback = None
        self._last_action_result = None
        self._action_in_progress = False
        self._action_start_time = 0.0
        self._last_action_feedback_time = 0.0  # Tempo do último feedback recebido
        self._action_timeout = 60.0  # Timeout padrão de 60 segundos desde último feedback

        # ==================================================================
        # SERVICE CLIENTS (CV_NODE)
        # ==================================================================
        self._cv_detection_client = create_client_from(self, Topics.Service.CV_DETECTION)
        self.get_logger().info(f"Service client criado: {Topics.Service.CV_DETECTION.name}")

        self._cv_record_client = create_client_from(self, Topics.Service.CV_RECORD_DETECTIONS)
        self.get_logger().info(f"Service client criado: {Topics.Service.CV_RECORD_DETECTIONS.name}")

        self._cv_anomaly_detection_client = create_client_from(self, Topics.Service.CV_ENABLE_ANOMALY)
        self.get_logger().info(f"Service client criado: {Topics.Service.CV_ENABLE_ANOMALY.name}")

        # Variáveis para controle de aproximação (usadas em estados APROXIMANDO/ESCANEANDO)
        self._detection_bbox_center = None    # Centro do bbox detectado [x, y]
        self._approach_arrival_time = 0.0     # Tempo de chegada na aproximação
        self._previous_inspection_position = None  # (lat, lon, alt) antes de aproximar

        # ==================================================================
        # SUBSCRIBERS
        # ==================================================================
        self.drone_state_sub = create_subscription_from(self, Topics.Interno.DRONE_STATE, self.drone_state_callback)
        self.dashboard_command_sub = create_subscription_from(self, Topics.Dashboard.MISSION_COMMANDS, self.dashboard_mission_command_callback)
        self.cv_detection_sub = create_subscription_from(self, Topics.Interno.CV_OBJECT_DETECTIONS, self.cv_detection_callback)

        # ==================================================================
        # TIMERS
        # ==================================================================
        # Timer principal da máquina de missão - executa a atualização e publicação do estado a cada 600ms
        self.mission_timer = self.create_timer(0.6, self.update_and_publish_mission_state)
        
        # Timer para verificação de saúde dos tópicos essenciais - executa a cada 2 segundos
        self.topic_health_timer = self.create_timer(2.0, self.check_essential_topics)
        
        self.get_logger().info("================ MISSION NODE PRONTO =============")

    # ==================================================================
    # MÉTODOS AUXILIARES DE CONFIGURAÇÃO
    # ==================================================================
    
    def get_missions_directory(self) -> str:
        """
        Obtém o diretório base para salvar dados das missões.
        Lê do parâmetro global do config params.yaml.
        
        Returns:
            Caminho do diretório de missões (padrão: ~/Drone_Inspetor_Missoes)
        """
        # TODO: Implementar leitura do params.yaml via rosparam ou arquivo direto
        # Por enquanto retorna o valor padrão
        return "~/Drone_Inspetor_Missoes"

    # ==================================================================
    # TIMERS - Funções Principais (executadas periodicamente)
    # ==================================================================

    def update_and_publish_mission_state(self):
        """
        Atualiza e publica o estado da máquina de missão a cada 0.6s.
        
        """
        self.mission_machine.tick()
        self.mission_ctx.publish()


    def check_essential_topics(self) -> bool:
        """
        Verifica a saúde dos tópicos essenciais.
        Executado por um timer a cada 2 segundos E chamado em verifica_mudanca_de_estado.
        
        Se algum tópico essencial estiver inativo:
        - Loga o erro
        - Cancela a action em andamento para parar o drone
        - Reseta a máquina de missão
        
        Returns:
            bool: True se todos os tópicos essenciais estão saudáveis, False caso contrário.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        unhealthy_topics = []
        
        for topic_name, topic_info in self.essencial_topics.items():
            time_since_last = current_time - topic_info["last_received"]
            
            # Se nunca recebeu (0.0), ignoramos na verificação de saúde por timeout
            # (O bloqueio de inicialização deve ser feito no verifica_mudanca_de_estado)
            if topic_info["last_received"] > 0.0 and time_since_last > self.topic_health_timeout:
                unhealthy_topics.append((topic_name, f"sem dados há {time_since_last:.1f}s"))
        
        if unhealthy_topics:
            unhealthy_list = ", ".join([
                f"{name} ({self.essencial_topics[name]['description']}): {reason}" 
                for name, reason in unhealthy_topics
            ])
            self.get_logger().error(
                f"TÓPICOS ESSENCIAIS INATIVOS: {unhealthy_list}. Cancelando ação em andamento e resetando máquina de missão...",
                throttle_duration_sec=5
            )
            
            # Cancela a action em andamento para parar o drone imediatamente
            self.cancel_current_action()
            
            # Reseta a máquina de missão
            self.mission_ctx.reset()
            
            return False
        
        return True

    def send_drone_action(self, command_dict):
        """
        Envia um comando para o drone_node usando DroneCommand Action.
        
        Args:
            command_dict: Dicionário com o comando a ser enviado.
            
        Returns:
            bool: True se o goal foi enviado com sucesso, False caso contrário.
        """
        # Verifica se já existe uma ação em progresso
        if self._action_in_progress:
            cmd = command_dict.get("command", "")
            self.get_logger().warn(f"Ignorando comando {cmd}: Já existe uma ação em progresso!")
            return False
        
        # Marca a ação como em progresso
        self._action_in_progress = True
        self._action_start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Verifica se o action server está disponível
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server /drone_inspetor/action/drone_command não disponível!")
            self._action_in_progress = False
            self._action_start_time = 0.0
            return False
            
        # Cria o goal
        goal_msg = DroneCommand.Goal()
        goal_msg.command = command_dict.get("command", "")
        
        # Parâmetros para GOTO - define NaN se não fornecido (indica "não especificado")
        goal_msg.lat = command_dict.get("lat", float('nan'))
        goal_msg.lon = command_dict.get("lon", float('nan'))
        goal_msg.alt = command_dict.get("alt", float('nan'))
        goal_msg.yaw = command_dict.get("yaw", float('nan'))
        
        # Foco para GOTO (use_focus=True): mantém yaw apontando ao foco durante o trajeto
        goal_msg.use_focus = bool(command_dict.get("use_focus", False))
        goal_msg.focus_lat = command_dict.get("focus_lat", float('nan'))
        goal_msg.focus_lon = command_dict.get("focus_lon", float('nan'))
        
        # Parâmetros para TAKEOFF
        if "altitude" in command_dict:
            goal_msg.altitude = command_dict["altitude"]
        elif "alt" in command_dict:
            goal_msg.altitude = command_dict["alt"]
        else:
            goal_msg.altitude = float('nan')

        self.get_logger().info(LogPrefix.drone_tx(goal_msg.command))
        
        # Envia o goal de forma assíncrona
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._action_feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True

    def _goal_response_callback(self, future):
        """
        Callback chamado quando o goal é aceito ou rejeitado.
        
        Args:
            future: Future com o resultado da requisição de goal
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejeitado pelo drone_node!")
            self._action_in_progress = False
            self._action_start_time = 0.0
            return
        
        self.get_logger().info("Goal aceito pelo drone_node, aguardando execução...")
        self._current_goal_handle = goal_handle
        
        # Configura callback para quando o resultado estiver disponível
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._action_result_callback)

    def _action_feedback_callback(self, feedback_msg):
        """
        Callback chamado quando feedback é recebido durante execução da action.
        Atualiza o timer de feedback para manter a action ativa enquanto houver progresso.
        
        Args:
            feedback_msg: Mensagem de feedback com progresso
        """
        feedback = feedback_msg.feedback
        self._last_action_feedback = feedback
        
        # Atualiza o tempo do último feedback (usado para timeout)
        self._last_action_feedback_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().debug(
            f"Feedback Action: estado = {feedback.state_name}, "
            f"distância = {feedback.distance_to_target:.2f}m, "
            f"progresso = {feedback.progress_percent:.1f}%",
            throttle_duration_sec=1.0
        )

    def _action_result_callback(self, future):
        """
        Callback chamado quando a action é completada.
        
        Args:
            future: Future com o resultado da action
        """
        result = future.result().result
        self._last_action_result = result
        self._action_in_progress = False
        self._action_start_time = 0.0
        self._last_action_feedback_time = 0.0
        self._current_goal_handle = None
        
        if result.success:
            self.get_logger().info(f"Action completada com sucesso: {result.message}")
            
            # Se o comando foi bem sucedido e estamos inspecionando, assumimos chegada ao waypoint
            if self.mission_ctx.state == MissionStateDescription.EXECUTANDO_INSPECIONANDO:
                self.get_logger().info("Waypoint alcançado (Action Success)! Iniciando contagem de tempo.")
                self.mission_ctx.ponto_de_inspecao_tempo_de_chegada = self.get_clock().now().nanoseconds / 1e9
                
        else:
            self.get_logger().warn(f"Action falhou: {result.message}")

    def cancel_current_action(self):
        """
        Cancela a action em andamento, se houver.
        
        Returns:
            bool: True se havia uma action para cancelar, False caso contrário.
        """
        if self._current_goal_handle is not None:
            self.get_logger().info("Cancelando action em andamento...")
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            return True
        return False

    def _cancel_done_callback(self, future):
        """
        Callback chamado quando o cancelamento é processado.
        
        Args:
            future: Future com o resultado do cancelamento
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Action cancelada com sucesso")
        else:
            self.get_logger().warn("Falha ao cancelar action")
    
    def _detection_response_callback(self, future):
        """
        Callback chamado quando a resposta do service de detecção CV é recebida.
        
        Args:
            future: Future com o resultado do service CVDetectionSRV
        """
        try:
            response = future.result()
            
            if response.success and len(response.bbox_center) >= 2:
                # Usa o centro do bounding box retornado pelo service
                bbox_center_x = response.bbox_center[0]
                bbox_center_y = response.bbox_center[1]
                
                self._detection_bbox_center = (bbox_center_x, bbox_center_y)
                
                self.get_logger().info(
                    f"🎯 Detecção bem sucedida! {response.message}, "
                    f"Confiança: {response.confidence:.2f}, "
                    f"BBox Center: ({bbox_center_x:.1f}, {bbox_center_y:.1f})"
                )
            else:
                self._detection_bbox_center = None
                self.get_logger().warn(
                    f"❌ {response.message if response.message else 'Objeto não detectado'}"
                )
                
        except Exception as e:
            self._detection_bbox_center = None
            self.get_logger().error(f"Erro ao processar resposta de detecção: {e}")

    def check_action_timeout(self) -> bool:
        """
        Verifica se a action em andamento excedeu o timeout desde o último feedback.
        Se excedeu, cancela a action e retorna True.
        
        Returns:
            bool: True se houve timeout e a action foi cancelada, False caso contrário.
        """
        if not self._action_in_progress:
            return False
        
        # Usa o tempo do último feedback para calcular o timeout
        # Se ainda não recebeu feedback, usa o tempo de início da action
        reference_time = self._last_action_feedback_time if self._last_action_feedback_time > 0 else self._action_start_time
        elapsed = (self.get_clock().now().nanoseconds / 1e9) - reference_time
        
        if elapsed > self._action_timeout:
            self.get_logger().error(
                f"TIMEOUT na action! ({elapsed:.1f}s desde último feedback > {self._action_timeout:.1f}s). Cancelando..."
            )
            self.cancel_current_action()
            return True
        
        return False

    def verifica_validade_do_comando(self, command_enum: DashboardMissionCommandDescription) -> tuple[bool, str]:
        """
        Valida se um comando do dashboard é permitido no estado atual.
        
        Args:
            command_enum: Enum do comando já convertido
            
        Returns:
            tuple: (can_execute, error_message)
                - can_execute: True se comando é permitido, False se não
                - error_message: Mensagem de erro se não permitido, string vazia se permitido
        """
        allowed_states = self.VALID_DASHBOARD_COMMANDS.get(command_enum, [])
        
        if not allowed_states:
            return False, f"Comando '{command_enum.name}' não está configurado em VALID_DASHBOARD_COMMANDS"
        
        current_state = self.mission_ctx.state
        if current_state in allowed_states:
            return True, ""
        else:
            allowed_names = [s.name for s in allowed_states]
            return False, (
                f"Comando '{command_enum.name}' não permitido no estado {current_state.name}. "
                f"Estados permitidos: {allowed_names}"
            )


    # ==================================================================
    # CALLBACKS DE EVENTOS
    # ==================================================================

    def dashboard_mission_command_callback(self, msg: DashboardMissionCommandMSG):
        """
        Processa comandos recebidos do dashboard para controlar a missão.
        Este é o callback principal que traduz comandos do Dashboard em ações da máquina de missão.
        
        Comandos suportados:
        - INICIAR_MISSAO: Seta on_mission=True e carrega missão
        - CANCELAR_MISSAO: Seta cancel_mission=True para tratamento no verifica_mudanca_de_estado
        
        Args:
            msg: Mensagem DashboardMissionCommandMSG do dashboard_node
        """
        command = msg.command
        
        # Converte o inteiro para o enum
        try:
            command_enum = DashboardMissionCommandDescription(command)
        except ValueError:
            self.get_logger().warn(f"Comando do Dashboard desconhecido recebido: {command}")
            return
        
        self.get_logger().info(f"<-- Comando do Dashboard recebido (DashboardMissionCommandMSG): {command_enum.name}")
        
        # Valida se o comando é permitido no estado atual
        can_execute, error_msg = self.verifica_validade_do_comando(command_enum)
        
        if not can_execute:
            self.get_logger().warn(f"Comando rejeitado: {error_msg}")
            return
        
        # Processa comandos usando match/case
        match command_enum:
            
            case DashboardMissionCommandDescription.INICIAR_MISSAO:
                mission_name = msg.mission
                
                # Valida e carrega a missão
                is_valid, error = self.mission_ctx.valida_missao(mission_name)
                if not is_valid:
                    self.get_logger().error(f"Missão inválida: {error}")
                    return
                
                # Seta flag on_mission para ser tratada no verifica_mudanca_de_estado
                self.mission_ctx.on_mission = True
                self.get_logger().info(f"Comando 'INICIAR_MISSAO' ACEITO. Missão: {mission_name}")
            
            case DashboardMissionCommandDescription.CANCELAR_MISSAO:
                self.get_logger().warn("Comando 'CANCELAR_MISSAO'")
                
                # Seta flag cancel_mission para ser tratada no verifica_mudanca_de_estado
                self.mission_ctx.cancel_mission = True
                


    def drone_state_callback(self, msg: DroneStateMSG):
        """
        Processa estado do drone recebido do drone_node.
        
        Args:
            msg: Mensagem DroneStateMSG do drone_node
        """
        # Atualiza timestamp do tópico essencial
        self.essencial_topics["drone_state"]["last_received"] = self.get_clock().now().nanoseconds / 1e9
        
        # Atualiza dados do drone através da classe
        self.drone.update_from_msg(msg)

    def cv_detection_callback(self, msg: CVDetectionMSG):
        """
        Processa detecções do cv_node durante a inspeção.
        
        Args:
            msg: Mensagem CVDetectionMSG do cv_node
        """
        # TODO: Implementar lógica de detecção de CV para inspeção
        pass



def main(args=None):
    """Função principal do nó."""
    import signal
    
    rclpy.init(args=args)
    mission_node = MissionNode()
    
    # Handler para SIGINT (Ctrl+C) - encerramento limpo
    def signal_handler(sig, frame):
        mission_node.get_logger().info("Encerrando mission_node...")
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(mission_node)
    except Exception:
        pass  # Ignora exceções durante shutdown
    finally:
        try:
            mission_node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
