"""Composição ROS da missão: domínio, clientes e FSM possuem responsabilidades próprias."""

import time
from dataclasses import asdict

from ament_index_python.packages import get_package_share_directory

from drone_inspetor.common.enums import DashboardMissionCommandDescription as Command
from drone_inspetor.missions.repository import MissionRepository
from drone_inspetor.nodes.mission_node.action_client import DroneActionClient
from drone_inspetor.nodes.mission_node.config import MissionConfig
from drone_inspetor.nodes.mission_node.cv_client import CVClient
from drone_inspetor.nodes.mission_node.drone_state_data import DroneStateData
from drone_inspetor.nodes.mission_node.fsm.mission.context import MissionFSMContext
from drone_inspetor.nodes.mission_node.fsm.mission.machine import MissionFSM
from drone_inspetor.nodes.mission_node.runtime import MissionRuntime
from drone_inspetor.nodes.mission_node.session import create_session_directory
from drone_inspetor.nodes.mission_node.journal import MissionJournal
from drone_inspetor.ros_interfaces import (
    Topics, create_client_from, create_publisher_from, create_subscription_from,
    make_action_client,
)

from drone_inspetor_msgs.msg import DashboardMissionCommandMSG, DroneStateMSG, MissionStateMSG

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rosidl_runtime_py.convert import message_to_ordereddict

from .fsm.mission.description import MissionFSMDescription as MS


class MissionNode(Node):
    """Adapta comandos/telemetria ROS e publica o único estado mantido pela FSM.

    A missão ordena ações; somente DroneNode publica comandos de voo no PX4.
    Callbacks neste nó usam o grupo padrão mutuamente exclusivo, portanto a FSM
    e o diário observam atualizações serializadas da sessão.
    """

    VALID_DASHBOARD_COMMANDS = {
        Command.INICIAR_MISSAO: (MS.PRONTO,),
        Command.CANCELAR_MISSAO: (
            MS.EXECUTANDO_ARMANDO, MS.EXECUTANDO_DECOLANDO, MS.EXECUTANDO_INSPECIONANDO,
            MS.EXECUTANDO_INSPECIONANDO_DETECTANDO, MS.EXECUTANDO_INSPECIONANDO_ESCANEANDO,
            MS.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO,
            MS.EXECUTANDO_INSPECIONANDO_FALHA, MS.INSPECAO_FINALIZADA,
        ),
    }

    def __init__(self, **node_options):
        """Monta repositório, clientes e timers após validar configuração."""
        super().__init__('mission_node', **node_options)
        self.config = MissionConfig.from_node(self)
        self.journal = MissionJournal(self.get_logger().error)
        self._journal_session_active = False
        self._last_drone_message = DroneStateMSG()
        self._rosout_sub = self.create_subscription(Log, '/rosout', self.record_rosout, 100)
        self.repository = MissionRepository(
            self.config.resolve_missions_file(get_package_share_directory('drone_inspetor')),
            takeoff_altitude_m=self.config.takeoff_altitude,
            dwell_s=self.config.tempo_de_permanencia)
        try:
            definitions = self.repository.load()
            self.get_logger().info(f'Missões validadas: {list(definitions)}')
        except (OSError, ValueError) as error:
            self.get_logger().error(f'Não foi possível carregar missões: {error}')

        self.drone = DroneStateData()
        self.last_telemetry_at = None
        self.actions = DroneActionClient(
            make_action_client(self, Topics.Action.DRONE_COMMAND), self.get_logger(),
            feedback_timeout=self.config.action_feedback_timeout,
            cancel_timeout=self.config.action_cancel_timeout)
        self.cv = CVClient(
            create_client_from(self, Topics.Service.CV_DETECTION),
            create_client_from(self, Topics.Service.CV_RECORD_DETECTIONS),
            create_client_from(self, Topics.Service.CV_ENABLE_ANOMALY), self.get_logger(),
            detection_timeout=self.config.detection_timeout,
            service_timeout=self.config.detection_service_timeout,
            control_timeout=self.config.cv_control_timeout)
        self.runtime = MissionRuntime(
            self.drone, self.actions, self.cv, self.config, self.get_logger(),
            self.ros_time, self.telemetry_healthy)
        self.mission_ctx = MissionFSMContext()
        self.mission_machine = MissionFSM(self.mission_ctx, self.runtime)
        self.mission_machine.register_all_states()
        self.mission_machine.transition_to(MS.DESATIVADO)

        self.mission_state_pub = create_publisher_from(self, Topics.Interno.MISSION_STATE)
        self.drone_state_sub = create_subscription_from(
            self, Topics.Interno.DRONE_STATE, self.drone_state_callback)
        self.dashboard_command_sub = create_subscription_from(
            self, Topics.Dashboard.MISSION_COMMANDS, self.dashboard_mission_command_callback)
        self.mission_timer = self.create_timer(
            self.config.mission_period, self.update_and_publish_mission_state)
        # O executor deve continuar verificando prazos mesmo quando /clock não avança.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.topic_health_timer = self.create_timer(
            min(0.5, self.config.topic_health_timeout / 2), self.check_essential_topics,
            clock=self.watchdog_clock)

    def ros_time(self):
        """Relógio da missão; timestamps e duração de inspeção acompanham /clock."""
        return self.get_clock().now().nanoseconds / 1e9

    def record_rosout(self, message):
        """Inclui transições, comandos e diagnósticos dos nós da aplicação."""
        if message.name in ('drone_node', 'mission_node', 'cv_node'):
            self.journal.record('rosout', self.ros_time(), node=message.name,
                                level=message.level, message=message.msg,
                                source_stamp_s=message.stamp.sec + message.stamp.nanosec / 1e9)

    def telemetry_healthy(self):
        """Saúde do tópico agregado, não das estimativas individuais do PX4.

        DroneNode verifica a validade da posição usada no controle. Aqui basta
        saber se seu status chega; tempo ROS zero não significa ausência.
        """
        if self.last_telemetry_at is None:
            return False
        age = time.monotonic() - self.last_telemetry_at
        return age <= self.config.topic_health_timeout

    def check_essential_topics(self):
        """Cancela e desativa a FSM, não somente o contexto, se telemetria expirar."""
        self.actions.poll()
        self.cv.poll()
        healthy = self.telemetry_healthy()
        if not healthy and self.mission_machine.current_state_id != MS.DESATIVADO:
            self.mission_machine.reset('Telemetria do drone expirada')
            self.publish_mission_state()
        return healthy

    def update_and_publish_mission_state(self):
        """Executa decisões e publica o estado efetivamente ativo."""
        self.mission_machine.tick()
        self.publish_mission_state()

    def publish_mission_state(self):
        """Publica dados tipados no contrato ROS público existente."""
        context = self.mission_ctx
        state = self.mission_machine.current_state_id
        message = MissionStateMSG()
        message.state = int(state)
        message.state_name = state.name
        message.on_mission = context.on_mission
        message.cancel_mission = context.cancel_mission
        message.mission_name = context.mission.name if context.mission else ''
        message.mission_folder_path = context.mission_folder_path
        message.tempo_de_permanencia = float(context.tempo_de_permanencia)
        message.takeoff_altitude = float(context.takeoff_altitude)
        message.ponto_de_inspecao_indice_atual = context.ponto_de_inspecao_indice_atual
        message.total_pontos_de_inspecao = context.total_pontos()
        message.ponto_de_inspecao_tempo_de_chegada = context.ponto_de_inspecao_tempo_de_chegada
        point = context.get_ponto_atual()
        if point is not None and point.inspection is not None:
            message.objeto_alvo = point.inspection.object_name
            message.tipos_anomalia = list(point.inspection.anomaly_types)
        self.mission_state_pub.publish(message)
        if self._journal_session_active:
            age = (None if self.last_telemetry_at is None else
                   time.monotonic() - self.last_telemetry_at)
            self.journal.observe(self.ros_time(), message_to_ordereddict(message),
                                 message_to_ordereddict(self._last_drone_message),
                                 context.failure_reason, age)
            if not context.mission_folder_path:
                self.journal.record('session_end', self.ros_time(), state=state.name)
                self._journal_session_active = False
                # Mantém o arquivo aberto para os últimos rosout já em trânsito;
                # snapshots cessam aqui. Fecha no próximo início ou no shutdown.

    def verifica_validade_do_comando(self, command):
        """Valida comandos contra a mesma FSM usada na publicação."""
        state = self.mission_machine.current_state_id
        if state not in self.VALID_DASHBOARD_COMMANDS.get(command, ()):
            return False, f'{command.name} não permitido em {state.name}'
        if command is Command.INICIAR_MISSAO and self.mission_ctx.on_mission:
            return False, 'Já existe uma sessão aguardando início'
        return True, ''

    def dashboard_mission_command_callback(self, message: DashboardMissionCommandMSG):
        """Valida primeiro, prepara artefatos depois e só então inicia a sessão."""
        try:
            command = Command(message.command)
        except ValueError:
            self.get_logger().warning(f'Comando de missão desconhecido: {message.command}')
            return
        valid, reason = self.verifica_validade_do_comando(command)
        if not valid:
            self.get_logger().warning(reason)
            return
        if command is Command.CANCELAR_MISSAO:
            self.journal.record('cancel_requested', self.ros_time())
            self.mission_ctx.cancel_mission = True
            return
        if not self.telemetry_healthy():
            self.get_logger().warning('Início recusado: telemetria não está recente')
            return
        try:
            definition = self.repository.get(message.mission)
            directory = create_session_directory(self.config.missions_directory)
        except (OSError, ValueError) as error:
            self.get_logger().error(f'Não foi possível iniciar missão: {error}')
            return
        self.mission_ctx.start(definition, directory)
        self.journal.start(directory, asdict(definition), asdict(self.config), self.ros_time())
        self._journal_session_active = True
        self.journal.record('drone_initial', self.ros_time(),
                            drone=message_to_ordereddict(self._last_drone_message))
        self.publish_mission_state()
        self.get_logger().info(f'Sessão iniciada: {definition.name}; pasta {directory}')

    def drone_state_callback(self, message: DroneStateMSG):
        """Registra recebimento em tempo monotônico e converte a telemetria."""
        self.drone.update_from_msg(message)
        self._last_drone_message = message
        self.last_telemetry_at = time.monotonic()

    def destroy_node(self):
        """Invalida callbacks e solicita encerramento dos recursos remotos."""
        self.actions.cancel()
        self.cv.stop_inspection()
        self.journal.record('node_shutdown', self.ros_time())
        self.journal.close()
        return super().destroy_node()


def main(args=None):
    """Executa a missão sem esconder falhas inesperadas durante operação."""
    rclpy.init(args=args)
    node = None
    try:
        node = MissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
