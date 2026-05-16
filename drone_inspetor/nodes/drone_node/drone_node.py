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
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, VehicleStatus

from drone_inspetor_msgs.msg import DroneStateMSG, LidarMSG, ObstaclesMSG

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
from drone_inspetor.nodes.drone_node.obstacles import DroneObstacle
from drone_inspetor.nodes.drone_node.trajectory_profile import TrajectoryProfile
from drone_inspetor.nodes.drone_node.trajectory import Trajectory

# Contexts e FSMs.
from drone_inspetor.nodes.drone_node.fsm.drone.context import DroneFSMContext
from drone_inspetor.nodes.drone_node.fsm.deslocamento.context import DeslocamentoFSMContext
from drone_inspetor.nodes.drone_node.fsm.drone.machine import DroneFSM
from drone_inspetor.nodes.drone_node.fsm.deslocamento.machine import DeslocamentoFSM

# Mixins (action server + comandos PX4).
from drone_inspetor.nodes.drone_node.action_server import DroneActionServerMixin
from drone_inspetor.nodes.drone_node.px4_commands import DronePX4CommandsMixin


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

        self._is_drone_node_shutting_down = False

        # =========================================================================================
        # SUBSISTEMAS DO NÓ (atributos diretos)
        # =========================================================================================
        # Telemetria PX4 (atualizada pelos callbacks /fmu/out/*).
        self.state_px4 = DroneStatePX4()
        # Buffers de detecção de obstáculos (atualizados pelos callbacks lidar/depth).
        self.obstacles = DroneObstacle()
        # Perfil trapezoidal global (reset a cada segmento novo de DESLOCANDO).
        self.trajectory_profile = TrajectoryProfile(
            vc=self.param_cruise_velocity,
            ad=self.param_travel_acceleration,
            ao=self.param_obstacle_deceleration,
            arrival_tol=self.param_arrival_position_tol,
            arrival_v_tol=self.param_arrival_velocity_tol,
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

        create_subscription_from(self, Topics.Interno.LIDAR_DATA, self.lidar_data_callback)
        create_subscription_from(self, Topics.Interno.DEPTH_OBSTACLE_DETECTIONS, self.depth_obstacles_callback)

        # =========================================================================================
        # ACTION SERVER (interface com mission_node)
        # =========================================================================================
        self._action_callback_group = ReentrantCallbackGroup()
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

    def tick_lifecycle_e_publish_state(self) -> None:
        """Tick da DroneFSM (lifecycle) + publicação do DroneStateMSG."""
        self.drone_fsm.tick()
        # Ao sair de EM_VOO, força a DeslocamentoFSM a voltar para PLANANDO.
        if (
            self.drone_fsm_context.state != DroneFSMDescription.EM_VOO
            and self.deslocamento_fsm.current_state_id != DeslocamentoFSMDescription.PLANANDO
        ):
            self.deslocamento_fsm.reset_to_planando()
        self.publish_drone_status()

    def tick_deslocamento_e_publish_setpoint(self) -> None:
        """
        50 Hz: tica a DeslocamentoFSM (apenas em EM_VOO) e publica TrajectorySetpoint.

        O PX4 exige ~50 Hz de setpoint enquanto offboard; mesmo em estados sem manobra
        ativa publicamos hover (gerado por `Trajectory.compute_hover()`).
        """
        if self.state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return
        if self.state_px4.global_position is None or self.state_px4.local_position is None:
            return

        if self.drone_fsm_context.state == DroneFSMDescription.EM_VOO:
            self.deslocamento_fsm.tick()

        setpoint = self.trajectory.create_setpoint_for_current_state()
        self.px4_trajectory_setpoint_pub.publish(setpoint)

    def px4_publish_offboard_control_mode(self) -> None:
        """Publica OffboardControlMode a 50 Hz quando o PX4 está em OFFBOARD."""
        if self.state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.px4_offboard_control_mode_pub.publish(msg)

    # =============================================================================================
    # DroneStateMSG
    # =============================================================================================
    def publish_drone_status(self) -> None:
        """Compõe e publica o snapshot do estado do drone."""
        dctx = self.drone_fsm_context
        sctx = self.deslocamento_fsm_context
        px4 = self.state_px4
        target = sctx.target_stack.current

        msg = DroneStateMSG()

        # Lifecycle
        msg.state = int(dctx.state)
        msg.state_name = dctx.state.name
        msg.state_duration_sec = round(dctx.now() - dctx.state_entry_time, 2)

        # Flags
        msg.is_armed = px4.is_armed
        msg.is_landed = px4.is_landed
        msg.is_on_trajectory = sctx.state != DeslocamentoFSMDescription.PLANANDO

        # Posição corrente
        if px4.local_position is not None:
            msg.current_local_x = px4.local_position.x
            msg.current_local_y = px4.local_position.y
            msg.current_local_z = -px4.local_position.z
        else:
            msg.current_local_x = msg.current_local_y = msg.current_local_z = 0.0

        if px4.global_position is not None:
            msg.current_latitude = px4.global_position.lat
            msg.current_longitude = px4.global_position.lon
            msg.current_altitude = px4.global_position.alt
        else:
            msg.current_latitude = msg.current_longitude = msg.current_altitude = 0.0

        # Yaw corrente
        yaw_norm = px4.current_yaw_deg_normalized
        msg.current_yaw_deg = yaw_norm if yaw_norm >= 0 else yaw_norm + 360
        msg.current_yaw_deg_normalized = yaw_norm
        msg.current_yaw_rad = px4.current_yaw_rad

        # HOME
        if px4.home_global_lat is not None:
            msg.home_global_lat = px4.home_global_lat
            msg.home_global_lon = px4.home_global_lon if px4.home_global_lon is not None else float('nan')
            msg.home_global_alt = px4.home_global_alt if px4.home_global_alt is not None else float('nan')
        else:
            msg.home_global_lat = msg.home_global_lon = msg.home_global_alt = float('nan')

        if px4.home_local_position is not None:
            msg.home_local_x = px4.home_local_position[0]
            msg.home_local_y = px4.home_local_position[1]
            msg.home_local_z = -px4.home_local_position[2]
        else:
            msg.home_local_x = msg.home_local_y = msg.home_local_z = float('nan')

        if px4.home_yaw_deg is not None:
            msg.home_yaw_deg = px4.home_yaw_deg
            msg.home_yaw_deg_normalized = px4.home_yaw_deg_normalized
            msg.home_yaw_rad = px4.home_yaw_rad
        else:
            msg.home_yaw_deg = msg.home_yaw_deg_normalized = msg.home_yaw_rad = float('nan')

        # Target (topo da pilha)
        if target is not None:
            msg.target_local_x = target.local_position[0]
            msg.target_local_y = target.local_position[1]
            msg.target_local_z = -target.local_position[2]
            msg.target_lat = target.latitude if target.latitude is not None else float('nan')
            msg.target_lon = target.longitude if target.longitude is not None else float('nan')
            msg.target_alt = target.altitude if target.altitude is not None else float('nan')
            msg.target_direction_yaw_deg = target.direction_yaw_deg if target.direction_yaw_deg is not None else float('nan')
            msg.target_direction_yaw_deg_normalized = target.direction_yaw_deg_normalized if target.direction_yaw_deg_normalized is not None else float('nan')
            msg.target_direction_yaw_rad = target.direction_yaw_rad if target.direction_yaw_rad is not None else float('nan')
            msg.target_final_yaw_deg = target.final_yaw_deg if target.final_yaw_deg is not None else float('nan')
            msg.target_final_yaw_deg_normalized = target.final_yaw_deg_normalized if target.final_yaw_deg_normalized is not None else float('nan')
            msg.target_final_yaw_rad = target.final_yaw_rad if target.final_yaw_rad is not None else float('nan')
            if target.focus_latitude is not None:
                msg.focus_lat = target.focus_latitude
                msg.focus_lon = target.focus_longitude if target.focus_longitude is not None else float('nan')
            else:
                msg.focus_lat = msg.focus_lon = float('nan')
            msg.focus_yaw_deg = msg.focus_yaw_deg_normalized = msg.focus_yaw_rad = float('nan')
        else:
            for field in (
                'target_local_x', 'target_local_y', 'target_local_z',
                'target_lat', 'target_lon', 'target_alt',
                'target_direction_yaw_deg', 'target_direction_yaw_deg_normalized', 'target_direction_yaw_rad',
                'target_final_yaw_deg', 'target_final_yaw_deg_normalized', 'target_final_yaw_rad',
                'focus_lat', 'focus_lon',
                'focus_yaw_deg', 'focus_yaw_deg_normalized', 'focus_yaw_rad',
            ):
                setattr(msg, field, float('nan'))

        # Última posição estática
        if sctx.last_static_position is not None:
            msg.last_static_position_x = sctx.last_static_position[0]
            msg.last_static_position_y = sctx.last_static_position[1]
            msg.last_static_position_z = -sctx.last_static_position[2]
        else:
            msg.last_static_position_x = msg.last_static_position_y = msg.last_static_position_z = float('nan')

        if sctx.last_static_yaw_deg is not None:
            msg.last_static_yaw_deg = sctx.last_static_yaw_deg
            msg.last_static_yaw_deg_normalized = sctx.last_static_yaw_deg_normalized
            msg.last_static_yaw_rad = sctx.last_static_yaw_rad
        else:
            msg.last_static_yaw_deg = msg.last_static_yaw_deg_normalized = msg.last_static_yaw_rad = float('nan')

        # Desvio em curso
        if target is not None and target.is_desvio:
            msg.has_trajectory_adjusted = True
            msg.trajectory_adjusted_x = target.local_position[0]
            msg.trajectory_adjusted_y = target.local_position[1]
            msg.trajectory_adjusted_z = -target.local_position[2]
        else:
            msg.has_trajectory_adjusted = False
            msg.trajectory_adjusted_x = msg.trajectory_adjusted_y = msg.trajectory_adjusted_z = float('nan')

        # Velocidade / aceleração
        msg.current_velocity_x = px4.current_velocity_x
        msg.current_velocity_y = px4.current_velocity_y
        msg.current_velocity_z = px4.current_velocity_z
        msg.current_acceleration_x = px4.current_acceleration_x
        msg.current_acceleration_y = px4.current_acceleration_y
        msg.current_acceleration_z = px4.current_acceleration_z

        self.drone_state_pub.publish(msg)

    # =============================================================================================
    # CALLBACKS PX4
    # =============================================================================================

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

    def px4_vehicle_local_position_callback(self, msg) -> None:
        px4 = self.state_px4
        px4.local_position = msg
        px4.current_velocity_x = getattr(msg, 'vx', 0.0)
        px4.current_velocity_y = getattr(msg, 'vy', 0.0)
        px4.current_velocity_z = getattr(msg, 'vz', 0.0)
        px4.current_acceleration_x = getattr(msg, 'ax', 0.0)
        px4.current_acceleration_y = getattr(msg, 'ay', 0.0)
        px4.current_acceleration_z = getattr(msg, 'az', 0.0)

    def px4_vehicle_global_position_callback(self, msg) -> None:
        self.state_px4.global_position = msg

    def px4_vehicle_attitude_callback(self, msg) -> None:
        from drone_inspetor.common.math_utils import normalize_yaw_deg
        px4 = self.state_px4
        px4.vehicle_attitude = msg
        q_w, q_x, q_y, q_z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        px4.current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        px4.current_yaw_deg = math.degrees(px4.current_yaw_rad)
        px4.current_yaw_deg_normalized = normalize_yaw_deg(px4.current_yaw_deg)

    def px4_land_detected_callback(self, msg) -> None:
        self.state_px4.is_landed = msg.landed

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

    def px4_battery_status_callback(self, msg) -> None:
        self.state_px4.battery_status = msg
        self.battery_status_pub.publish(msg)

    def lidar_data_callback(self, msg: LidarMSG) -> None:
        """Callback para dados brutos do LiDAR — LidarObstacle processa e gera flags."""
        self.obstacles.update_from_lidar(msg)

    def depth_obstacles_callback(self, msg: ObstaclesMSG) -> None:
        """Callback para flags de obstáculo da câmera depth (ObstaclesMSG)."""
        self.obstacles.update_from_depth(msg)

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
    except (KeyboardInterrupt, ExternalShutdownException, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
