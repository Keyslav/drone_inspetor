"""Adapta o planejador local e a trajetória S-curve aos setpoints PX4 NED.

O PX4 fecha as malhas de posição/velocidade. Esta camada entrega referências
coerentes de posição, velocidade e aceleração, com feedforward nas duas últimas.
"""

import math
import time

from px4_msgs.msg import TrajectorySetpoint, VehicleStatus

from drone_inspetor.navigation.motion import speed_for_clearance, terminal_speed_limit
from drone_inspetor.navigation.obstacles import LocalPlanner, wrap
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import (
    DeslocamentoFSMDescription as TS,
)

_ZERO = (0.0, 0.0, 0.0)


class Trajectory:
    """Mantém continuidade de referência e decide quando parar ou desviar."""

    def __init__(self, node, drone_fsm_context, deslocamento_fsm_context, trajectory_profile):
        self.node = node
        self.drone_fsm_context = drone_fsm_context
        self.deslocamento_fsm_context = deslocamento_fsm_context
        self.trajectory_profile = trajectory_profile
        self.config = node.navigation_config
        self.planner = LocalPlanner(
            cruise=node.param_cruise_velocity, braking=node.param_obstacle_deceleration,
            jerk=self.config.jerk_limit, reaction_time=self.config.reaction_time,
            planning_distance=self.config.planning_distance,
            detour_distance=self.config.detour_distance,
            near_speed=self.config.near_obstacle_velocity,
        )
        self.reset()

    def reset(self):
        self.trajectory_profile.reset()
        self._last_time = None
        self.last_tick_elapsed = 0.
        self._dt = self.node.param_trajectory_timer_period
        self._takeoff_target = None
        self._takeoff_yaw = None
        self._yaw_reference = None
        self._blocked_since = None
        self._tracking_stall_since = None
        self._detours = 0
        self.navigation_error = None
        self.last_decision = None
        self._terminal_cap = self.trajectory_profile.cruise

    def begin_command(self):
        """Limpa o diagnóstico e o histórico somente no início de uma nova ação."""
        self.navigation_error = None
        self._blocked_since = None
        self._tracking_stall_since = None
        self._detours = 0
        self._takeoff_target = None
        self._takeoff_yaw = None

    def begin_tick(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        elapsed = self.node.param_trajectory_timer_period if self._last_time is None else now - self._last_time
        self._last_time = now
        self.last_tick_elapsed = elapsed
        # Atrasos do executor dentro do orçamento de reação não significam um
        # reinício do relógio. Nunca recuperar o tempo perdido com um salto da
        # referência; acima do orçamento, interromper e iniciar a frenagem.
        maximum_gap = min(self.config.reaction_time, self.config.telemetry_timeout)
        if elapsed < 0 or elapsed > maximum_gap:
            self.navigation_error = (
                'Relógio de trajetória interrompido ou reiniciado '
                f'(intervalo {elapsed:.3f}s, limite {maximum_gap:.3f}s)'
            )
        self._dt = max(0., min(elapsed, 0.1))

    @property
    def measured_position(self):
        pos = self.node.state_px4.local_position
        return pos.x, pos.y, pos.z

    @property
    def measured_velocity(self):
        state = self.node.state_px4
        return state.current_velocity_x, state.current_velocity_y, state.current_velocity_z

    @property
    def reference_position(self):
        profile = self.trajectory_profile
        if profile.origin is None:
            return tuple(self.deslocamento_fsm_context.last_static_position or self.measured_position)
        return tuple(o + d * profile.position for o, d in zip(profile.origin, profile.direction))

    @property
    def reference_stopped(self):
        """Frenagem da referência concluída; a parada medida é verificada à parte."""
        return self.trajectory_profile.stopped

    @property
    def stopped(self):
        return (self.reference_stopped
                and math.sqrt(sum(v * v for v in self.measured_velocity))
                <= self.node.param_arrival_velocity_tol)

    @property
    def settled(self):
        """Pronto para mudar de segmento: parado e próximo da referência mantida."""
        return (self.stopped and math.dist(self.measured_position, self.reference_position)
                <= self.node.param_arrival_position_tol)

    def create_setpoint_for_current_state(self):
        pos, vel, acc, yaw = self.compute_for_current_state()
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.position = list(pos)
        msg.velocity = list(vel)
        msg.acceleration = list(acc)
        msg.yaw = float(yaw)
        return msg

    def compute_for_current_state(self):
        if self.node.state_px4.local_position is None:
            raise RuntimeError('Sem posição válida para gerar referência')
        if self.node.state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return self.compute_hover()
        if self.navigation_error:
            return self.compute_hover()
        if self.drone_fsm_context.state == DS.DECOLANDO:
            return self.compute_vertical_takeoff()
        if self.drone_fsm_context.state == DS.EM_VOO:
            state = self.deslocamento_fsm_context.state
            if state == TS.DESLOCANDO:
                return self.compute_deslocando()
            if state in (TS.GIRANDO_INICIO, TS.GIRANDO_FIM):
                return self.compute_girando(state == TS.GIRANDO_FIM)
        return self.compute_hover()

    def compute_hover(self, yaw_target=None):
        """Freia a referência antes de manter a posição estática, inclusive no STOP."""
        context = self.deslocamento_fsm_context
        profile = self.trajectory_profile
        yaw = context.last_static_yaw_rad if yaw_target is None else yaw_target
        if yaw is None:
            yaw = self.node.state_px4.current_yaw_rad
        if profile.target is not None:
            pos, vel, acc = self._tick_profile(0.)
            context.last_static_position = list(pos)
            if self.stopped:
                profile.reset()
            return pos, vel, acc, self._yaw(yaw)
        position = context.last_static_position or self.measured_position
        return tuple(position), _ZERO, _ZERO, self._yaw(yaw)

    def compute_vertical_takeoff(self):
        if self._takeoff_target is None:
            x, y, _ = self.measured_position
            home = self.node.state_px4.home_local_position
            home_z = home[2] if home is not None else self.measured_position[2]
            self._takeoff_target = (x, y, home_z - self.drone_fsm_context.takeoff_altitude)
            self._takeoff_yaw = (self.node.state_px4.current_yaw_rad if self._yaw_reference is None
                                 else self._yaw_reference)
        self._start_if_needed(self._takeoff_target)
        # O lidar horizontal não observa o teto. Decolagem exige volume superior
        # livre verificado pelo operador; a velocidade vertical tem limite próprio.
        cap = min(self.config.vertical_velocity, self._arrival_cap())
        pos, vel, acc = self._tick_profile(self._tracking_cap(cap))
        return pos, vel, acc, self._yaw(self._takeoff_yaw)

    def compute_girando(self, use_final_yaw):
        target = self.deslocamento_fsm_context.target_stack.current
        desired = None
        # A FSM só inicia o giro após confirmar a parada. Oscilações posteriores
        # da velocidade medida em hover não devem restaurar o yaw anterior a
        # cada amostra; a referência translacional continua parada.
        if target is not None and self.trajectory_profile.stopped:
            desired = target.final_yaw_rad if use_final_yaw else target.direction_yaw_rad
        return self.compute_hover(yaw_target=desired)

    def prepare_next_segment(self):
        """Escolhe desvio observado antes de girar para um trecho já bloqueado.

        Falta de cobertura não impede o giro: ele pode colocar o corredor no FOV.
        A autorização para transladar continua sendo reavaliada a cada setpoint.
        """
        target = self.deslocamento_fsm_context.target_stack.current
        if target is None or not self.settled or self.navigation_error:
            return
        reference = self.reference_position
        now = time.monotonic()
        decision = self._avoidance_decision(reference, tuple(target.local_position), now)
        if decision.detour is not None:
            dx, dy = target.local_position[0] - reference[0], target.local_position[1] - reference[1]
            horizon = min(math.hypot(dx, dy), self.config.planning_distance)
            hit = self.node.navigation_sensors.map.hit_clearance(
                reference, math.atan2(dy, dx), horizon, now)
            # Sem retorno que bloqueie a rota, primeiro girar para observá-la;
            # não tratar o setor atrás do LiDAR como um obstáculo confirmado.
            if hit < horizon - .05:
                self._accept_detour(decision.detour, reference)

    def _avoidance_decision(self, reference, target, now):
        profile = self.trajectory_profile
        self.node.navigation_sensors.map.radius = (
            self.config.vehicle_radius + self.config.obstacle_margin
            + math.dist(self.measured_position, reference))
        self.last_decision = self.planner.evaluate(
            self.node.navigation_sensors.map, reference, target,
            math.sqrt(sum(v * v for v in self.measured_velocity)), 0., now,
            reference_speed=profile.velocity, reference_acceleration=profile.accel,
            allow_detour=self._detours < self.config.max_detours,
        )
        return self.last_decision

    def _accept_detour(self, point, reference):
        context = self.deslocamento_fsm_context
        if context.target_stack.is_loop_candidate(point):
            self.navigation_error = 'Desvio repetido; rota local sem progresso'
            return False
        self._detours += 1
        context.target_stack.push_desvio(list(point))
        context.last_static_position = list(reference)
        self.trajectory_profile.reset()
        self.node.get_logger().info(f'Desvio {self._detours}: {point}')
        return True

    def compute_deslocando(self):
        context = self.deslocamento_fsm_context
        target = context.target_stack.current
        if target is None:
            return self.compute_hover()
        target_position = tuple(target.local_position)
        self._start_if_needed(target_position)
        profile = self.trajectory_profile
        position = self.measured_position
        reference = self.reference_position
        if math.dist(position, reference) > self.config.tracking_error_limit:
            self.navigation_error = 'Erro de seguimento excedeu o limite; navegação interrompida'
            return self.compute_hover()
        now = time.monotonic()
        decision = self._avoidance_decision(reference, target_position, now)
        cap = decision.speed_limit
        # Um desvio só substitui a direção depois de referência E veículo parados
        # e próximos um do outro; não zera velocidade no meio de uma curva.
        if decision.detour is not None and self.settled:
            if self._accept_detour(decision.detour, reference):
                self.node.deslocamento_fsm.transition_to(TS.PLANANDO)
            return self.compute_hover()
        vertical_ratio = abs(profile.direction[2])
        if vertical_ratio > 1e-6:
            cap = min(cap, self.config.vertical_velocity / vertical_ratio)
        if profile.direction[2] > 1e-6:
            distance = self.node.navigation_sensors.descent_clearance(now)
            # Converte distância vertical em distância ao longo do segmento.
            cap = min(cap, speed_for_clearance(
                distance / vertical_ratio, profile.cruise, profile.deceleration,
                profile.jerk, self.config.reaction_time, max(0., profile.accel)))
        cap = self._tracking_cap(min(cap, self._arrival_cap()))
        self._observe_blocked(cap, now, decision.reason)
        pos, vel, acc = self._tick_profile(cap)
        if target.focus_local_position is not None:
            dx = target.focus_local_position[0] - position[0]
            dy = target.focus_local_position[1] - position[1]
            desired_yaw = math.atan2(dy, dx)
        else:
            desired_yaw = target.direction_yaw_rad
        if desired_yaw is None:
            desired_yaw = self.node.state_px4.current_yaw_rad
        return pos, vel, acc, self._yaw(desired_yaw)

    def _observe_blocked(self, cap, now, reason):
        if cap > 1e-5:
            self._blocked_since = None
        elif self._blocked_since is None:
            self._blocked_since = now
        elif now - self._blocked_since >= self.config.blocked_timeout:
            messages = {
                'sensor_stale': 'Sensor de navegação sem dados válidos por tempo excessivo',
                'braking_for_detour': 'Parada não estabilizou a tempo para iniciar o desvio',
                'detour': 'Posição não estabilizou a tempo para iniciar o desvio',
                'blocked': 'Nenhum corredor observado disponível por tempo excessivo',
            }
            self.navigation_error = messages.get(
                reason, 'Limite de movimento permaneceu em zero por tempo excessivo')

    def _start_if_needed(self, target):
        profile = self.trajectory_profile
        if profile.target != tuple(target):
            # Entre segmentos a referência anterior está parada. Usar velocidade
            # medida (ruído/deriva) aqui criaria um salto de feedforward no giro.
            origin = (self.reference_position if profile.target is not None else
                      self.deslocamento_fsm_context.last_static_position or self.measured_position)
            profile.start_segment(origin, target, v_initial=0.)
            self._terminal_cap = profile.cruise

    def _arrival_cap(self):
        """Antecipação terminal com alvo fixo e limite que só diminui no segmento."""
        profile = self.trajectory_profile
        remaining_measured = sum((target - position) * direction
                                 for target, position, direction in
                                 zip(profile.target, self.measured_position, profile.direction))
        remaining = min(profile.length - profile.position, remaining_measured)
        limit = terminal_speed_limit(
            remaining, profile.cruise, profile.deceleration, profile.jerk,
            self.config.reaction_time, self.config.arrival_approach_velocity,
            self.config.arrival_settling_time, max(0., profile.accel))
        # Correções/ruído do estimador não devem fazer reacelerar na aproximação.
        self._terminal_cap = min(self._terminal_cap, limit)
        return self._terminal_cap

    def _tracking_cap(self, cap):
        """Reserva margem de seguimento também para o erro transversal."""
        profile = self.trajectory_profile
        displacement = tuple(r - p for r, p in zip(self.reference_position, self.measured_position))
        lead = sum(delta * direction for delta, direction in zip(displacement, profile.direction))
        lateral_squared = max(0., sum(delta * delta for delta in displacement) - lead * lead)
        available_lead = math.sqrt(max(0., self.config.reference_lead_limit ** 2 - lateral_squared))
        measured = sum(v * d for v, d in zip(self.measured_velocity, profile.direction))
        governed = max(0., measured + self.config.tracking_gain * (available_lead - lead))
        if lead > .9 * self.config.reference_lead_limit and abs(measured) < .1:
            now = time.monotonic()
            if self._tracking_stall_since is None:
                self._tracking_stall_since = now
            elif now - self._tracking_stall_since > self.config.blocked_timeout:
                self.navigation_error = 'Veículo não acompanhou a referência de trajetória'
        else:
            self._tracking_stall_since = None
        return min(cap, governed)

    def _tick_profile(self, cap):
        if self._dt == 0:
            profile = self.trajectory_profile
            return (self.reference_position, tuple(d * profile.velocity for d in profile.direction),
                    tuple(d * profile.accel for d in profile.direction))
        return self.trajectory_profile.tick(
            self._dt, self.measured_position, cap, self.measured_velocity)

    def _yaw(self, target):
        if self._yaw_reference is None:
            self._yaw_reference = self.node.state_px4.current_yaw_rad
        step = math.radians(self.config.yaw_rate_deg) * self._dt
        delta = wrap(target - self._yaw_reference)
        self._yaw_reference = wrap(self._yaw_reference + max(-step, min(step, delta)))
        return self._yaw_reference

    def global_to_local_position(self, latitude, longitude, altitude):
        """GPS absoluto para NED; HOME pode ter deslocamento local não nulo."""
        from drone_inspetor.common.math_utils import global_to_ned_offset
        state = self.node.state_px4
        if state.home_global_lat is None or state.home_local_position is None:
            raise ValueError('HOME indisponível para converter destino GPS')
        north, east, down = global_to_ned_offset(
            state.home_global_lat, state.home_global_lon, state.home_global_alt,
            latitude, longitude, altitude,
        )
        home = state.home_local_position
        return [home[0] + north, home[1] + east, home[2] + down]
