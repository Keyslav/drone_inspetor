# =================================================================================================
# drone_node.py
# =================================================================================================
# NÓ DE CONTROLE DE BAIXO NÍVEL DO DRONE (os "músculos")
# =================================================================================================
# Única interface com o PX4. Traduz comandos de alto nível (DroneCommand Action vindo do
# mission_node) em mensagens PX4 e expõe a telemetria via DroneStateMSG.
#
# Hospeda DUAS FSMs internas, cada uma com seu próprio Context:
#
#   1. DroneFSM (lifecycle do drone) + DroneFSMContext
#        - 6 estados, tickada a `fsm_timer_period` (0.5s).
#        - OFFBOARD_DESATIVADO, POUSADO_DESARMADO, POUSADO_ARMADO,
#          DECOLANDO, EM_VOO, EMERGENCIA.
#
#   2. DeslocamentoFSM (fases de manobra em voo) + DeslocamentoFSMContext
#        - 4 estados, tickada a `trajectory_timer_period` (0.02s) APENAS quando lifecycle == EM_VOO.
#        - PLANANDO (hover), GIRANDO_INICIO, DESLOCANDO, GIRANDO_FIM.
#
# Subsistemas comuns (state_px4, obstacles) ficam como atributos do DroneNode em vez
# de viverem em algum dos Contexts. Os Contexts acessam-nos via `self.node.<atributo>`.
#
# Cálculo de trajetória: classe `Trajectory` (composta no nó) escolhe o método correto com
# base no estado das duas FSMs e gera o TrajectorySetpoint a 50 Hz.
# =================================================================================================

import math
import time
from threading import RLock
from functools import wraps
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, VehicleStatus

from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription
from drone_inspetor.common.log_colors import LogPrefix
from drone_inspetor.common.param_utils import load_param
from drone_inspetor.ros_interfaces import (
    Topics,
    create_publisher_from,
    create_subscription_from,
    make_action_server,
)

# Subsistemas do nó.
from drone_inspetor.nodes.drone_node.px4_state import DroneStatePX4
from drone_inspetor.nodes.drone_node.trajectory_profile import TrajectoryProfile
from drone_inspetor.nodes.drone_node.trajectory import Trajectory
from drone_inspetor.nodes.drone_node.telemetry import drone_state_message
from drone_inspetor.navigation.config import NavigationConfig
from drone_inspetor.navigation.poses import PoseHistory
from drone_inspetor.nodes.drone_node.navigation_sensors import NavigationSensors

# Contexts e FSMs.
from drone_inspetor.nodes.drone_node.fsm.drone.context import DroneFSMContext
from drone_inspetor.nodes.drone_node.fsm.deslocamento.context import DeslocamentoFSMContext
from drone_inspetor.nodes.drone_node.fsm.drone.machine import DroneFSM
from drone_inspetor.nodes.drone_node.fsm.deslocamento.machine import DeslocamentoFSM

# Mixins (action server + comandos PX4).
from drone_inspetor.nodes.drone_node.action_server import DroneActionServerMixin
from drone_inspetor.nodes.drone_node.px4_commands import DronePX4CommandsMixin


def control_snapshot(callback):
    """Atualiza um snapshot de telemetria sem intercalar um despacho de comando."""
    @wraps(callback)
    def serialized(self, *args, **kwargs):
        with self._control_lock:
            return callback(self, *args, **kwargs)
    return serialized


class DroneNode(DroneActionServerMixin, DronePX4CommandsMixin, Node):
    """
    DroneNode — interface única com o PX4 + controle das FSMs internas.

    Comunicação:
        - Recebe comandos do mission_node via DroneCommand Action.
        - Recebe telemetria do PX4 via /fmu/out/*.
        - Publica setpoints para o PX4 via /fmu/in/*.
        - Publica DroneStateMSG (status agregado) para o mission_node e dashboard.
    """

    def __init__(self):
        super().__init__("drone_node")
        self.get_logger().info("================ INICIALIZANDO DRONE NODE ==============")

        # =========================================================================================
        # PARÂMETROS ROS
        # =========================================================================================
        self.param_cruise_velocity = load_param(self, "cruise_velocity", 3.0)
        self.param_travel_acceleration = load_param(self, "travel_acceleration", 1.0)
        self.param_obstacle_deceleration = load_param(self, "obstacle_deceleration", 2.0)
        self.param_arrival_position_tol = load_param(self, "arrival_position_tol", 0.2)
        self.param_arrival_velocity_tol = load_param(self, "arrival_velocity_tol", 0.1)

        self.param_trajectory_timer_period = load_param(self, "trajectory_timer_period", 0.02)
        self.param_offboard_control_mode_timer_period = load_param(
            self, "offboard_control_mode_timer_period", 0.02
        )
        self.param_fsm_timer_period = load_param(self, "fsm_timer_period", 0.5)

        self.param_px4_target_system_id = load_param(self, 'px4_target_system_id', 1)
        if not isinstance(self.param_px4_target_system_id, int) or not 1 <= self.param_px4_target_system_id <= 255:
            raise ValueError('px4_target_system_id deve estar entre 1 e 255')
        self.navigation_config = NavigationConfig.from_node(self)
        self.pose_history = PoseHistory()
        self._control_lock = RLock()
        self._position_received = None
        self._setpoint_published = None
        self._is_drone_node_shutting_down = False

        # =========================================================================================
        # SUBSISTEMAS DO NÓ (atributos diretos)
        # =========================================================================================
        # Telemetria PX4 (atualizada pelos callbacks /fmu/out/*).
        self.state_px4 = DroneStatePX4()
        # Perfil S-curve global com limites de velocidade, aceleração e jerk.
        self.trajectory_profile = TrajectoryProfile(
            vc=self.param_cruise_velocity,
            ad=self.param_travel_acceleration,
            ao=self.param_obstacle_deceleration,
            arrival_tol=self.param_arrival_position_tol,
            arrival_v_tol=self.param_arrival_velocity_tol,
            jerk=self.navigation_config.jerk_limit,
        )

        # =========================================================================================
        # CONTEXTS + FSMs
        # =========================================================================================
        # DroneFSM (lifecycle) + DroneFSMContext.
        self.drone_fsm_context = DroneFSMContext(self)
        self.drone_fsm = DroneFSM(self.drone_fsm_context, self)
        self.drone_fsm.register_all_states()
        self.drone_fsm.transition_to(DroneFSMDescription.OFFBOARD_DESATIVADO)

        # DeslocamentoFSM (manobra) + DeslocamentoFSMContext.
        self.deslocamento_fsm_context = DeslocamentoFSMContext(self)
        self.deslocamento_fsm = DeslocamentoFSM(self.deslocamento_fsm_context, self)
        self.deslocamento_fsm.register_all_states()
        self.deslocamento_fsm.transition_to(DeslocamentoFSMDescription.PLANANDO)

        # =========================================================================================
        # TRAJECTORY (composição, não mixin)
        # =========================================================================================
        self.trajectory = Trajectory(
            node=self,
            drone_fsm_context=self.drone_fsm_context,
            deslocamento_fsm_context=self.deslocamento_fsm_context,
            trajectory_profile=self.trajectory_profile,
        )

        # Estado interno do mixin de comandos PX4 (ACK tracking).
        self.init_px4_commands_state()

        # =========================================================================================
        # SUBSCRIBERS (telemetria PX4 + sensores)
        # =========================================================================================
        create_subscription_from(self, Topics.PX4.VEHICLE_STATUS, self.px4_vehicle_status_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_COMMAND_ACK, self.px4_command_ack_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_LOCAL_POSITION, self.px4_vehicle_local_position_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_GLOBAL_POSITION, self.px4_vehicle_global_position_callback)
        create_subscription_from(self, Topics.PX4.HOME_POSITION, self.px4_home_position_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_ATTITUDE, self.px4_vehicle_attitude_callback)
        create_subscription_from(self, Topics.PX4.VEHICLE_LAND_DETECTED, self.px4_land_detected_callback)
        create_subscription_from(self, Topics.PX4.BATTERY_STATUS, self.px4_battery_status_callback)


        # =========================================================================================
        # ACTION SERVER (interface com mission_node)
        # =========================================================================================
        self.navigation_sensors = NavigationSensors(self, self.navigation_config)
        self._action_callback_group = ReentrantCallbackGroup()
        self.init_action_server_state()
        self._action_server = make_action_server(
            self,
            Topics.Action.DRONE_COMMAND,
            execute_callback=self.execute_drone_command_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_callback_group,
        )

        # =========================================================================================
        # PUBLISHERS (PX4 + status para mission_node/dashboard)
        # =========================================================================================
        self.px4_vehicle_command_pub = create_publisher_from(self, Topics.PX4.VEHICLE_COMMAND)
        self.px4_offboard_control_mode_pub = create_publisher_from(self, Topics.PX4.OFFBOARD_CONTROL_MODE)
        self.px4_trajectory_setpoint_pub = create_publisher_from(self, Topics.PX4.TRAJECTORY_SETPOINT)
        self.drone_state_pub = create_publisher_from(self, Topics.Interno.DRONE_STATE)
        self.battery_status_pub = create_publisher_from(self, Topics.Interno.DRONE_BATTERY_STATUS)

        # =========================================================================================
        # TIMERS
        # =========================================================================================
        self.create_timer(
            self.param_offboard_control_mode_timer_period,
            self.px4_publish_offboard_control_mode,
        )
        self.create_timer(
            self.param_trajectory_timer_period,
            self.tick_deslocamento_e_publish_setpoint,
        )
        self.create_timer(
            self.param_fsm_timer_period,
            self.tick_lifecycle_e_publish_state,
        )

        self.get_logger().info("================ DRONE NODE PRONTO ================")

    # =============================================================================================
    # TIMERS — loops periódicos
    # =============================================================================================

    def telemetry_fresh(self):
        return (self._position_received is not None
                and time.monotonic() - self._position_received <= self.navigation_config.telemetry_timeout)

    def tick_lifecycle_e_publish_state(self):
        """Serializa transições com o despacho de comandos do ActionServer."""
        with self._control_lock:
            self.drone_fsm.tick()
            if self.drone_fsm_context.state != DroneFSMDescription.EM_VOO:
                if self.deslocamento_fsm.current_state_id != DeslocamentoFSMDescription.PLANANDO:
                    self.deslocamento_fsm.reset_to_planando()
            self.publish_drone_status()

    def tick_deslocamento_e_publish_setpoint(self):
        """Pré-publica hover antes do OFFBOARD e avança movimento só quando ativo."""
        with self._control_lock:
            if not self.telemetry_fresh():
                # Não mantém prova de vida com estado inválido. O failsafe de
                # perda de OFFBOARD configurado no PX4 passa a ser responsável.
                return
            offboard = self.state_px4.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            if not offboard:
                # AUTO/POSCTL usam o mesmo tópico uORB de setpoint dentro do PX4.
                # Publicar hover aqui disputa o controle com o modo nativo e pode
                # impedir LAND/RTL de terminar. No solo desarmado, a pré-publicação
                # continua preparando a próxima ativação OFFBOARD.
                if self.state_px4.is_armed:
                    self._setpoint_published = None
                    return
                self.trajectory.reset()
                self.deslocamento_fsm_context.store_static_position()
            self.trajectory.begin_tick()
            if offboard and self.drone_fsm_context.state == DroneFSMDescription.EM_VOO:
                self.deslocamento_fsm.tick()
            try:
                setpoint = self.trajectory.create_setpoint_for_current_state()
            except (ValueError, RuntimeError) as error:
                self.trajectory.navigation_error = str(error)
                self.get_logger().error(f'Falha de trajetória: {error}', throttle_duration_sec=2.)
                return
            self.px4_trajectory_setpoint_pub.publish(setpoint)
            self._setpoint_published = time.monotonic()

    def px4_publish_offboard_control_mode(self):
        """Prova de vida prévia ao OFFBOARD; p/v/a são referências no setpoint."""
        if (not self.telemetry_fresh() or self._setpoint_published is None
                or time.monotonic() - self._setpoint_published > 0.2):
            return
        msg = OffboardControlMode()
        msg.position = True
        # position seleciona a malha externa. Feedforward v/a permanece no
        # TrajectorySetpoint, sem selecionar modos de controle concorrentes.
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.px4_offboard_control_mode_pub.publish(msg)

    # =============================================================================================
    # DroneStateMSG
    # =============================================================================================
    def publish_drone_status(self):
        """Publica uma cópia; conversões do contrato ROS ficam no mapper."""
        self.drone_state_pub.publish(drone_state_message(
            self.state_px4, self.drone_fsm_context, self.deslocamento_fsm_context,
            self.get_clock().now().nanoseconds / 1e9,
        ))

    # =============================================================================================
    # CALLBACKS PX4
    # =============================================================================================

    @control_snapshot
    def px4_vehicle_status_callback(self, msg) -> None:
        px4 = self.state_px4
        was_armed = px4.is_armed
        is_now_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        if is_now_armed and not was_armed:
            self.get_logger().info(LogPrefix.px4_rx("Motores: ARMADOS"), throttle_duration_sec=2)
        elif not is_now_armed and was_armed:
            self.get_logger().info(LogPrefix.px4_rx("Motores: DESARMADOS"), throttle_duration_sec=2)
        px4.is_armed = is_now_armed

        was_offboard = px4.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        is_now_offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if is_now_offboard and not was_offboard:
            self.get_logger().info(LogPrefix.px4_rx("Modo OFFBOARD ativado."), throttle_duration_sec=2)
        elif not is_now_offboard and was_offboard:
            self.get_logger().info(LogPrefix.px4_rx("Modo OFFBOARD desativado."), throttle_duration_sec=2)
        px4.nav_state = msg.nav_state

    @control_snapshot
    def px4_vehicle_local_position_callback(self, msg) -> None:
        px4 = self.state_px4
        if not all(math.isfinite(value) for value in (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)):
            return
        if not (msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid):
            return
        self._position_received = time.monotonic()
        px4.local_position = msg
        self.pose_history.add(self.get_clock().now().nanoseconds / 1e9,
                              (msg.x, msg.y, msg.z), px4.current_yaw_rad)
        px4.current_velocity_x = getattr(msg, 'vx', 0.0)
        px4.current_velocity_y = getattr(msg, 'vy', 0.0)
        px4.current_velocity_z = getattr(msg, 'vz', 0.0)
        px4.current_acceleration_x = getattr(msg, 'ax', 0.0)
        px4.current_acceleration_y = getattr(msg, 'ay', 0.0)
        px4.current_acceleration_z = getattr(msg, 'az', 0.0)

    @control_snapshot
    def px4_vehicle_global_position_callback(self, msg) -> None:
        self.state_px4.global_position = msg

    @control_snapshot
    def px4_vehicle_attitude_callback(self, msg) -> None:
        from drone_inspetor.common.math_utils import normalize_yaw_deg
        px4 = self.state_px4
        px4.vehicle_attitude = msg
        if not all(math.isfinite(value) for value in msg.q):
            return
        q_w, q_x, q_y, q_z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        px4.current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        px4.current_yaw_deg = math.degrees(px4.current_yaw_rad)
        px4.current_yaw_deg_normalized = normalize_yaw_deg(px4.current_yaw_deg)

    @control_snapshot
    def px4_land_detected_callback(self, msg) -> None:
        self.state_px4.is_landed = msg.landed

    @control_snapshot
    def px4_home_position_callback(self, msg) -> None:
        px4 = self.state_px4
        px4.home_position = msg
        px4.home_global_lat = msg.lat
        px4.home_global_lon = msg.lon
        px4.home_global_alt = msg.alt
        px4.home_local_position = [msg.x, msg.y, msg.z]

        yaw_rad = msg.yaw
        yaw_deg = math.degrees(yaw_rad)
        yaw_norm = ((yaw_deg + 180.0) % 360.0) - 180.0
        yaw_0_360 = yaw_norm if yaw_norm >= 0 else yaw_norm + 360
        px4.home_yaw_rad = yaw_rad
        px4.home_yaw_deg = yaw_0_360
        px4.home_yaw_deg_normalized = yaw_norm

        self.get_logger().info(
            LogPrefix.px4_rx(
                f"HOME atualizado: GPS=[{px4.home_global_lat:.6f}, {px4.home_global_lon:.6f}, "
                f"{px4.home_global_alt:.2f}m], Yaw={px4.home_yaw_deg:.1f}°"
            )
        )

    @control_snapshot
    def px4_battery_status_callback(self, msg) -> None:
        self.state_px4.battery_status = msg
        self.battery_status_pub.publish(msg)

    # =============================================================================================
    # Shutdown
    # =============================================================================================

    def destroy_node(self) -> None:
        self._is_drone_node_shutting_down = True
        self.get_logger().info("Encerrando drone_node...")
        super().destroy_node()


# =================================================================================================
# main()
# =================================================================================================

def main(args=None):
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    node = DroneNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
