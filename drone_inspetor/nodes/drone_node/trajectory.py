# =================================================================================================
# trajectory.py — classe Trajectory
# =================================================================================================
# Classe Trajectory: encapsula TODAS as funções de cálculo de trajetória do DroneNode.
# Substitui a antiga pasta trajectories/ (Strategy Pattern com várias subclasses de
# BaseTrajectory) por uma única classe com métodos de cálculo. Evoluirá com o tempo
# conforme novas manobras forem necessárias.
#
# Responsabilidades:
#     1. Despachar o cálculo correto conforme o estado das duas FSMs (lifecycle + manobra).
#     2. Gerar a mensagem TrajectorySetpoint (PX4) a cada tick de 50 Hz.
#     3. Integrar com o TrajectoryProfile (perfil trapezoidal de velocidade).
#     4. Calcular yaw incremental (rotação suave) e yaw apontando para foco.
#
# Modos de cálculo (um por método público `compute_*`):
#     compute_hover            → setpoint na last_static_position (idle).
#     compute_vertical_takeoff → subida vertical até takeoff_altitude.
#     compute_girando          → rotação no eixo, posição fixa, yaw incremental.
#     compute_deslocando       → translação até o target via TrajectoryProfile + yaw.
# =================================================================================================

import math
from typing import TYPE_CHECKING

from px4_msgs.msg import TrajectorySetpoint

from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription
from drone_inspetor.nodes.drone_node.trajectory_profile import TrajectoryProfile

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode
    from drone_inspetor.nodes.drone_node.fsm.drone.context import DroneFSMContext
    from drone_inspetor.nodes.drone_node.fsm.deslocamento.context import DeslocamentoFSMContext


# Zero 3D constante — reutilizado em vários retornos.
_ZERO3 = (0.0, 0.0, 0.0)


class Trajectory:
    """
    Calculadora de trajetória do DroneNode.

    Composta no `DroneNode` (não é mixin). Recebe referências ao nó, aos dois Contexts
    e ao TrajectoryProfile no construtor para evitar acoplamento via herança.
    """

    # Período do tick de trajetória — deve casar com `param_trajectory_timer_period`.
    _TRAJ_DT = 0.02

    def __init__(
        self,
        node: 'DroneNode',
        drone_fsm_context: 'DroneFSMContext',
        deslocamento_fsm_context: 'DeslocamentoFSMContext',
        trajectory_profile: TrajectoryProfile,
    ):
        # Referências (read-only do ponto de vista desta classe).
        self.node = node
        self.drone_fsm_context = drone_fsm_context
        self.deslocamento_fsm_context = deslocamento_fsm_context
        self.trajectory_profile = trajectory_profile

    # =============================================================================================
    # Dispatch principal (chamado pelo loop de 50 Hz)
    # =============================================================================================

    def create_setpoint_for_current_state(self) -> TrajectorySetpoint:
        """
        Calcula o TrajectorySetpoint do ciclo atual, escolhendo o método de cálculo
        baseado no estado das duas FSMs.

        Returns:
            TrajectorySetpoint pronto para publicação no tópico /fmu/in/trajectory_setpoint.
        """
        pos, vel, acc, yaw = self.compute_for_current_state()

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.position[0], msg.position[1], msg.position[2] = pos
        msg.velocity[0], msg.velocity[1], msg.velocity[2] = vel
        msg.acceleration[0], msg.acceleration[1], msg.acceleration[2] = acc
        msg.yaw = yaw if yaw is not None else float('nan')
        return msg

    def compute_for_current_state(self) -> tuple:
        """
        Decide qual método compute_* invocar com base no estado das duas FSMs.

        Returns:
            (pos_xyz, vel_xyz, acc_xyz, yaw_rad).
        """
        # Sem telemetria de posição: setpoint nulo (origem com vel/acc zero, yaw indefinido).
        if self.node.state_px4.local_position is None:
            self.node.get_logger().warn(
                "Trajetória: posição local indisponível.", throttle_duration_sec=2.0
            )
            return (0.0, 0.0, 0.0), _ZERO3, _ZERO3, None

        lifecycle = self.drone_fsm_context.state

        # (1) Lifecycle DECOLANDO: subida vertical até takeoff_altitude.
        if lifecycle == DroneFSMDescription.DECOLANDO:
            return self.compute_vertical_takeoff()

        # (2) Lifecycle EM_VOO: DeslocamentoFSM dita o método.
        if lifecycle == DroneFSMDescription.EM_VOO:
            return self._compute_for_deslocamento_state()

        # (3) Lifecycle em solo / inativo: hover na referência estática.
        return self.compute_hover()

    def _compute_for_deslocamento_state(self) -> tuple:
        """Sub-dispatcher quando lifecycle == EM_VOO: escolhe conforme estado da DeslocamentoFSM."""
        ds = self.deslocamento_fsm_context.state
        DS = DeslocamentoFSMDescription

        if ds == DS.PLANANDO:
            return self.compute_hover()
        if ds == DS.GIRANDO_INICIO:
            return self.compute_girando(use_final_yaw=False)
        if ds == DS.DESLOCANDO:
            return self.compute_deslocando()
        if ds == DS.GIRANDO_FIM:
            return self.compute_girando(use_final_yaw=True)
        # Fallback defensivo.
        return self.compute_hover()

    # =============================================================================================
    # COMPUTAÇÕES POR MODO
    # =============================================================================================

    def compute_hover(self) -> tuple:
        """
        Hover estático na referência (last_static_position + last_static_yaw).
        Fallback seguro quando não há manobra ativa.
        """
        self._reset_profile_if_active()
        dctx = self.deslocamento_fsm_context

        if dctx.last_static_position is not None:
            pos = (
                dctx.last_static_position[0],
                dctx.last_static_position[1],
                dctx.last_static_position[2],
            )
        else:
            lp = self.node.state_px4.local_position
            pos = (lp.x, lp.y, lp.z) if lp is not None else _ZERO3

        yaw = (
            dctx.last_static_yaw_rad
            if dctx.last_static_yaw_rad is not None
            else self.node.state_px4.current_yaw_rad
        )
        return pos, _ZERO3, _ZERO3, yaw

    def compute_vertical_takeoff(self) -> tuple:
        """
        Subida vertical: X/Y atuais, Z = home_z - takeoff_altitude. Yaw mantido (HOME ou atual).
        """
        self._reset_profile_if_active()
        cur = self.node.state_px4.local_position
        if cur is None:
            return _ZERO3, _ZERO3, _ZERO3, None

        home_z = (
            self.node.state_px4.home_local_position[2]
            if self.node.state_px4.home_local_position is not None
            else 0.0
        )
        target_z = home_z - self.drone_fsm_context.takeoff_altitude

        yaw = (
            self.node.state_px4.home_yaw_rad
            if self.node.state_px4.home_yaw_rad is not None
            else self.node.state_px4.current_yaw_rad
        )
        return (cur.x, cur.y, target_z), _ZERO3, _ZERO3, yaw

    def compute_girando(self, use_final_yaw: bool) -> tuple:
        """
        Rotação no eixo: posição fixa (last_static), yaw incrementando até alvo.

        Args:
            use_final_yaw: True → alvo é target.final_yaw_rad (estado GIRANDO_FIM).
                           False → alvo é target.direction_yaw_rad (estado GIRANDO_INICIO).
        """
        self._reset_profile_if_active()
        dctx = self.deslocamento_fsm_context

        if dctx.last_static_position is not None:
            pos = tuple(dctx.last_static_position)
        else:
            lp = self.node.state_px4.local_position
            pos = (lp.x, lp.y, lp.z) if lp is not None else _ZERO3

        target = dctx.target_stack.current
        if target is None:
            return pos, _ZERO3, _ZERO3, self.node.state_px4.current_yaw_rad

        yaw_alvo = target.final_yaw_rad if use_final_yaw else target.direction_yaw_rad
        if yaw_alvo is None:
            return pos, _ZERO3, _ZERO3, self.node.state_px4.current_yaw_rad

        next_yaw = self._calculate_incremental_yaw(
            self.node.state_px4.current_yaw_rad, yaw_alvo, dctx.yaw_step_deg
        )
        return pos, _ZERO3, _ZERO3, next_yaw

    def compute_deslocando(self) -> tuple:
        """
        Translação até o target ativo via TrajectoryProfile (perfil trapezoidal).
        Yaw: foco (se houver) ou direção da rota.
        """
        dctx = self.deslocamento_fsm_context
        target = dctx.target_stack.current
        cur = self.node.state_px4.local_position

        if target is None or cur is None:
            return self.compute_hover()

        target_pos = tuple(target.local_position)
        cur_pos = (cur.x, cur.y, cur.z)
        next_pos, next_vel, next_acc = self._advance_profile(cur_pos, target_pos)

        if target.focus_local_position is not None:
            yaw = self._calculate_focus_yaw(cur.x, cur.y)
        elif target.direction_yaw_rad is not None:
            yaw = target.direction_yaw_rad
        else:
            yaw = self.node.state_px4.current_yaw_rad

        return next_pos, next_vel, next_acc, yaw

    # =============================================================================================
    # Helpers de yaw
    # =============================================================================================

    def _calculate_incremental_yaw(
        self, current_yaw_rad: float, target_yaw_rad: float, max_step_deg: float
    ) -> float:
        """Próximo passo incremental de yaw (rotação suave) em direção ao alvo."""
        from drone_inspetor.common.math_utils import yaw_step_toward
        next_deg = yaw_step_toward(
            math.degrees(current_yaw_rad),
            math.degrees(target_yaw_rad),
            max_step_deg,
        )
        return math.radians(next_deg)

    def _calculate_focus_yaw(self, from_x: float, from_y: float) -> float:
        """
        Yaw apontando de (from_x, from_y) para o ponto de foco do target ativo.
        Retorna 0.0 se não houver target ou foco.
        """
        target = self.deslocamento_fsm_context.target_stack.current
        if target is None or target.focus_local_position is None:
            return 0.0
        dx = target.focus_local_position[0] - from_x
        dy = target.focus_local_position[1] - from_y
        return math.atan2(dy, dx)

    # =============================================================================================
    # Integração com TrajectoryProfile
    # =============================================================================================

    def _advance_profile(self, current_pos: tuple, target_pos: tuple) -> tuple:
        """
        Avança o TrajectoryProfile um tick rumo ao target. Detecta segmento novo
        (entrada em DESLOCANDO ou troca de target) e reinicia o perfil quando necessário.

        Returns:
            (pos_xyz, vel_xyz, acc_xyz).
        """
        profile = self.trajectory_profile
        is_new_segment = (
            not profile.is_active()
            or profile.target is None
            or self._distance_squared(profile.target, target_pos) > 1e-4
        )
        if is_new_segment:
            v_initial = self._current_speed_along(target_pos, current_pos)
            profile.start_segment(current_pos, target_pos, v_initial=v_initial)

        from drone_inspetor.nodes.drone_node.obstacle_avoidance import get_velocity_cap_from_obstacles
        v_target_obs = get_velocity_cap_from_obstacles(
            self.node.obstacles, self.node.param_cruise_velocity
        )
        return profile.tick(self._TRAJ_DT, current_pos, v_target_obs=v_target_obs)

    def _reset_profile_if_active(self) -> None:
        """Encerra o segmento corrente — usado fora de DESLOCANDO."""
        if self.trajectory_profile.is_active():
            self.trajectory_profile.reset()

    def _current_speed_along(self, target_pos: tuple, current_pos: tuple) -> float:
        """Velocidade real do drone projetada na direção current→target (não-negativa)."""
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        dz = target_pos[2] - current_pos[2]
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm < 1e-6:
            return 0.0
        ux, uy, uz = dx / norm, dy / norm, dz / norm
        px4 = self.node.state_px4
        return max(0.0, px4.current_velocity_x * ux + px4.current_velocity_y * uy + px4.current_velocity_z * uz)

    @staticmethod
    def _distance_squared(a: tuple, b: tuple) -> float:
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2

    # =============================================================================================
    # Conversão GPS → frame local NED (utilitário público)
    # =============================================================================================

    def global_to_local_position(self, target_lat, target_lon, target_alt):
        """
        Converte coordenadas globais (GPS) para frame local NED relativo ao HOME do PX4.

        Returns:
            [x, y, z] em NED (Z negativo = altitude positiva), ou None se HOME indefinido.
        """
        px4 = self.node.state_px4
        if px4.home_global_lat is None:
            self.node.get_logger().warn(
                "global_to_local_position: HOME não disponível. Arme o drone primeiro.",
                throttle_duration_sec=5.0,
            )
            return None
        from drone_inspetor.common.math_utils import global_to_local_offset
        dx, dy, dz = global_to_local_offset(
            px4.home_global_lat, px4.home_global_lon, px4.home_global_alt,
            target_lat, target_lon, target_alt,
        )
        return [dx, dy, -dz]
