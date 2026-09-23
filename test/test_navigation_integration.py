"""Trajetória e FSM reais com telemetria/relógio determinísticos, sem veículo."""

import math
from types import SimpleNamespace as NS

import pytest
from px4_msgs.msg import VehicleStatus

from drone_inspetor.navigation.config import NavigationConfig
from drone_inspetor.navigation.obstacles import ObstacleMap
from drone_inspetor.nodes.drone_node.fsm.deslocamento.context import DeslocamentoFSMContext
from drone_inspetor.nodes.drone_node.fsm.deslocamento.machine import DeslocamentoFSM
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription as TS
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.drone_node.trajectory import Trajectory
from drone_inspetor.nodes.drone_node.trajectory_profile import TrajectoryProfile
from test_navigation_obstacles import update


class Harness:
    def __init__(self, monkeypatch, position=(0., 0., -3.)):
        self.time = 1.
        self.navigation_config = NavigationConfig(yaw_stabilization_seconds=.02)
        self.param_trajectory_timer_period = .02
        self.param_cruise_velocity = 3.
        self.param_travel_acceleration = 1.
        self.param_obstacle_deceleration = 2.
        self.param_arrival_position_tol = .2
        self.param_arrival_velocity_tol = .1
        self.state_px4 = NS(
            local_position=NS(x=position[0], y=position[1], z=position[2]),
            current_velocity_x=0., current_velocity_y=0., current_velocity_z=0.,
            current_yaw_rad=0., current_yaw_deg_normalized=0.,
            nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            home_local_position=[0., 0., 0.], home_global_lat=1.,
            home_global_lon=1., home_global_alt=10.,
        )
        self.navigation_sensors = NS(map=ObstacleMap(), descent_clearance=lambda now: 50.)
        self.drone_fsm_context = NS(state=DS.EM_VOO, takeoff_altitude=3.)
        self.deslocamento_fsm_context = DeslocamentoFSMContext(self)
        self.deslocamento_fsm = DeslocamentoFSM(self.deslocamento_fsm_context, self)
        self.deslocamento_fsm.register_all_states()
        self.deslocamento_fsm.transition_to(TS.PLANANDO)
        self.trajectory_profile = TrajectoryProfile(3., 1., 2.)
        self.trajectory = Trajectory(self, self.drone_fsm_context,
                                     self.deslocamento_fsm_context, self.trajectory_profile)
        monkeypatch.setattr('drone_inspetor.nodes.drone_node.trajectory.time.monotonic', lambda: self.time)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.time * 1e9)))

    def get_logger(self):
        return NS(info=lambda *a, **kw: None, warn=lambda *a, **kw: None,
                  error=lambda *a, **kw: None, debug=lambda *a, **kw: None)

    def goto(self, position, **kwargs):
        self.deslocamento_fsm_context.target_stack.push_missao(list(position), **kwargs)
        self.trajectory.begin_command()

    def tick(self, obstacles=(), follow=True, scan_enabled=True):
        self.time += .02
        if scan_enabled:
            update(self.navigation_sensors.map, obstacles,
                   self.trajectory.measured_position, self.state_px4.current_yaw_rad, self.time)
        self.trajectory.begin_tick()
        if self.drone_fsm_context.state == DS.EM_VOO:
            self.deslocamento_fsm.tick()
        pos, vel, acc, yaw = self.trajectory.compute_for_current_state()
        if follow:
            self.state_px4.local_position = NS(x=pos[0], y=pos[1], z=pos[2])
            self.state_px4.current_velocity_x, self.state_px4.current_velocity_y, self.state_px4.current_velocity_z = vel
            self.state_px4.current_yaw_rad = yaw
            self.state_px4.current_yaw_deg_normalized = math.degrees(yaw)
        return pos, vel, acc, yaw


@pytest.mark.parametrize('destination', [(0., 0., -6.), (1., 0., -3.), (20., 0., -3.), (8., 4., -5.)])
def test_adapter_completes_vertical_short_and_diagonal_moves(monkeypatch, destination):
    node = Harness(monkeypatch)
    node.goto(destination)
    for _ in range(6000):
        node.tick()
        assert node.trajectory.navigation_error is None
        if node.deslocamento_fsm_context.target_stack.is_empty:
            break
    assert node.deslocamento_fsm_context.target_stack.is_empty
    assert math.dist(node.trajectory.measured_position, destination) < .01
    assert node.trajectory.stopped


def test_yaw_rate_has_units_of_degrees_per_second(monkeypatch):
    node = Harness(monkeypatch)
    node.goto((0., 5., -3.))
    previous = 0.
    for _ in range(200):
        *_, yaw = node.tick()
        assert abs(yaw - previous) <= math.radians(30) * .02 + 1e-8
        previous = yaw
    assert yaw == pytest.approx(math.pi / 2, abs=.01)


def test_hover_velocity_noise_does_not_reverse_initial_rotation(monkeypatch):
    node = Harness(monkeypatch)
    node.goto((0., 5., -3.))
    node.tick()
    assert node.deslocamento_fsm_context.state == TS.GIRANDO_INICIO
    previous = node.state_px4.current_yaw_rad
    for _ in range(170):
        node.state_px4.current_velocity_x = .15
        *_, yaw = node.tick()
        assert 0. <= yaw - previous <= math.radians(30) * .02 + 1e-8
        previous = yaw
    assert yaw == pytest.approx(math.pi / 2, abs=.01)
    assert node.deslocamento_fsm_context.state == TS.GIRANDO_INICIO
    node.state_px4.current_velocity_x = 0.
    node.tick()
    assert node.deslocamento_fsm_context.state == TS.DESLOCANDO


@pytest.mark.parametrize('position', [(0., 0., -3.), (2.424, 1.684, -3.)])
def test_known_blockage_selects_detour_before_initial_or_resumed_rotation(monkeypatch, position):
    node = Harness(monkeypatch, position=position)
    node.goto((12., 0., -3.))
    original = node.deslocamento_fsm_context.target_stack.current
    node.tick(obstacles=[(6., 0.)], follow=False)
    target = node.deslocamento_fsm_context.target_stack.current
    assert node.trajectory._detours == 1
    assert target is not original
    assert node.deslocamento_fsm_context.state == TS.GIRANDO_INICIO
    assert target.direction_yaw_rad == pytest.approx(math.atan2(
        target.local_position[1] - position[1], target.local_position[0] - position[0]))
    # A missão continua embaixo do desvio; não se perde o destino solicitado.
    node.deslocamento_fsm_context.target_stack.pop()
    assert node.deslocamento_fsm_context.target_stack.current is original


def test_unobserved_rear_corridor_can_be_inspected_by_rotating_before_detouring(monkeypatch):
    node = Harness(monkeypatch)
    node.goto((-12., 0., -3.))
    original = node.deslocamento_fsm_context.target_stack.current
    node.navigation_sensors.map.update('lidar', [math.inf] * 181, -math.pi / 2,
                                      math.pi / 180, .1, 15., (0., 0., -3.), 0., node.time)
    node.tick(scan_enabled=False, follow=False)
    assert node.trajectory._detours == 0
    assert node.deslocamento_fsm_context.target_stack.current is original
    assert node.deslocamento_fsm_context.state == TS.GIRANDO_INICIO
    assert abs(original.direction_yaw_rad) == pytest.approx(math.pi)
    assert node.trajectory_profile.stopped


def test_aligned_yaw_waits_for_position_to_settle_before_translation(monkeypatch):
    node = Harness(monkeypatch)
    node.deslocamento_fsm_context.store_static_position()
    node.goto((10., 0., -3.))
    node.tick()
    assert node.deslocamento_fsm_context.state == TS.GIRANDO_INICIO
    node.state_px4.local_position.y = .4
    for _ in range(20):
        position, velocity, _, _ = node.tick(follow=False)
        assert position == (0., 0., -3.) and velocity == (0., 0., 0.)
        assert node.deslocamento_fsm_context.state == TS.GIRANDO_INICIO
    node.state_px4.local_position.y = 0.
    node.tick()
    assert node.deslocamento_fsm_context.state == TS.DESLOCANDO


def test_hover_reference_reports_commanded_position_when_profile_is_reset(monkeypatch):
    node = Harness(monkeypatch)
    node.deslocamento_fsm_context.store_static_position()
    node.state_px4.local_position.x = .3
    position, *_ = node.trajectory.compute_hover()
    assert position == (0., 0., -3.)
    assert node.trajectory.reference_position == position


def test_obstacle_is_circumvented_without_leaving_original_target(monkeypatch):
    node = Harness(monkeypatch)
    destination = (12., 0., -3.)
    node.goto(destination)
    minimum = math.inf
    max_stack = 0
    for _ in range(6000):
        pos, *_ = node.tick(obstacles=[(5., 0.)])
        minimum = min(minimum, math.hypot(pos[0] - 5., pos[1]))
        max_stack = max(max_stack, node.deslocamento_fsm_context.target_stack.size)
        assert node.trajectory.navigation_error is None
        if node.deslocamento_fsm_context.target_stack.is_empty:
            break
    assert max_stack >= 2
    assert minimum > 1.2  # obstáculo 0.4 + veículo/margem 0.8
    assert math.dist(node.trajectory.measured_position, destination) < .01
    assert node.deslocamento_fsm_context.target_stack.is_empty


def test_stale_scan_brakes_without_discontinuous_feedforward(monkeypatch):
    node = Harness(monkeypatch)
    node.goto((40., 0., -3.))
    for _ in range(250):
        node.tick()
    assert node.trajectory_profile.velocity > 2.
    previous_acceleration = node.trajectory_profile.accel
    for _ in range(400):
        node.tick(scan_enabled=False)
        acceleration = node.trajectory_profile.accel
        assert abs(acceleration - previous_acceleration) <= .040001
        previous_acceleration = acceleration
    assert node.trajectory.stopped
    assert not node.deslocamento_fsm_context.target_stack.is_empty


def test_stop_uses_existing_reference_until_braking_finishes(monkeypatch):
    node = Harness(monkeypatch)
    node.goto((40., 0., -3.))
    for _ in range(250):
        node.tick()
    node.deslocamento_fsm_context.reset()
    node.deslocamento_fsm.reset_to_planando()
    old_acceleration = node.trajectory_profile.accel
    old_position = node.trajectory.measured_position
    for _ in range(300):
        position, velocity, acceleration, _ = node.tick()
        assert abs(acceleration[0] - old_acceleration) <= .040001
        assert position[0] >= old_position[0] - 1e-8
        old_acceleration, old_position = acceleration[0], position
    assert node.trajectory.stopped
    assert velocity == (0., 0., 0.)


def test_takeoff_keeps_xy_and_uses_same_smooth_profile(monkeypatch):
    node = Harness(monkeypatch, (2., 4., 1.))
    node.state_px4.home_local_position = [2., 4., 1.]
    node.drone_fsm_context.state = DS.DECOLANDO
    for _ in range(1000):
        pos, vel, acc, _ = node.tick()
        assert pos[:2] == (2., 4.)
        assert abs(vel[2]) <= 1.000001
        if node.trajectory_profile.is_done():
            break
    assert node.trajectory_profile.is_done()
    assert pos == pytest.approx((2., 4., -2.))


def test_home_conversion_preserves_nonzero_local_offset(monkeypatch):
    node = Harness(monkeypatch)
    node.state_px4.home_local_position = [10., 20., 5.]
    assert node.trajectory.global_to_local_position(1., 1., 12.) == pytest.approx([10., 20., 3.])


@pytest.mark.parametrize('lag', [.08, .2])
def test_reference_with_lagged_position_velocity_controller(monkeypatch, lag):
    """Planta simplificada com atraso: evita usar seguimento perfeito como única prova."""
    node = Harness(monkeypatch)
    target = (14., 0., -3.)
    node.goto(target)
    position = list(node.trajectory.measured_position)
    velocity, acceleration = [0.] * 3, [0.] * 3
    maximum_error = 0.
    for _ in range(6000):
        reference, speed, feedforward, yaw = node.tick(follow=False)
        for axis in range(3):
            desired = (feedforward[axis] + 3. * (speed[axis] - velocity[axis])
                       + 3. * (reference[axis] - position[axis]))
            acceleration[axis] += .02 / lag * (desired - acceleration[axis])
            position[axis] += velocity[axis] * .02 + acceleration[axis] * .02 ** 2 / 2
            velocity[axis] += acceleration[axis] * .02
        state = node.state_px4
        state.local_position = NS(x=position[0], y=position[1], z=position[2])
        state.current_velocity_x, state.current_velocity_y, state.current_velocity_z = velocity
        state.current_yaw_rad, state.current_yaw_deg_normalized = yaw, math.degrees(yaw)
        maximum_error = max(maximum_error, math.dist(reference, position))
        assert node.trajectory.navigation_error is None
        if node.deslocamento_fsm_context.target_stack.is_empty:
            break
    assert maximum_error < .3
    assert node.deslocamento_fsm_context.target_stack.is_empty
    assert math.dist(position, target) < .2
    assert math.sqrt(sum(v * v for v in velocity)) < .1


def test_tracking_error_aborts_instead_of_advancing_forever(monkeypatch):
    node = Harness(monkeypatch)
    node.goto((30., 0., -3.))
    for _ in range(2000):
        node.tick(follow=False)
        if node.trajectory.navigation_error:
            break
    assert 'referência' in node.trajectory.navigation_error


def test_approach_preserves_cruise_and_arrives_with_drag_and_velocity_integrator(monkeypatch):
    """Modelo reduzido dos ganhos SIH; arrasto exige integral durante cruzeiro.

    Não modela EKF/atitude completos. A planta reproduz a falha anterior de
    ultrapassagem >1m, ao contrário do teste com seguimento perfeito.
    """
    node = Harness(monkeypatch)
    node.goto((40., 0., -3.))
    position = velocity = acceleration = integral = force = 0.
    maximum_error = overshoot = cruise_time = 0.
    previous_acceleration = 0.
    for _ in range(6000):
        reference, speed, feedforward, yaw = node.tick(follow=False)
        error = speed[0] + .95 * (reference[0] - position) - velocity
        integral += .4 * error * .02
        desired_force = feedforward[0] + 1.8 * error + integral - .2 * acceleration
        force += (desired_force - force) * .02 / .08
        acceleration = force - velocity  # massa=1 kg, arrasto linear=1 kg/s
        position += velocity * .02 + .5 * acceleration * .02 ** 2
        velocity += acceleration * .02
        node.state_px4.local_position = NS(x=position, y=0., z=-3.)
        node.state_px4.current_velocity_x = velocity
        node.state_px4.current_yaw_rad = yaw
        node.state_px4.current_yaw_deg_normalized = math.degrees(yaw)
        maximum_error = max(maximum_error, abs(position - reference[0]))
        overshoot = max(overshoot, position - 40.)
        cruise_time += .02 if abs(speed[0] - 3.) < .01 else 0.
        assert abs(feedforward[0] - previous_acceleration) <= .040001
        previous_acceleration = feedforward[0]
        assert node.trajectory.navigation_error is None
        if node.deslocamento_fsm_context.target_stack.is_empty:
            break
    assert node.deslocamento_fsm_context.target_stack.is_empty
    assert maximum_error < 1.
    assert overshoot < .6
    assert cruise_time > 2.
    assert abs(position - 40.) < .2 and abs(velocity) < .1


def test_terminal_approach_does_not_disable_hard_tracking_limit(monkeypatch):
    node = Harness(monkeypatch)
    node.goto((3., 0., -3.))
    for _ in range(100):
        node.tick()
    node.state_px4.local_position.x += 1.1
    node.tick(follow=False)
    assert 'Erro de seguimento' in node.trajectory.navigation_error


@pytest.mark.parametrize('target,position,velocity', [
    ((30., 0., -3.), (-.3, .6, -3.), (2., 0., 0.)),
    ((0., 30., -3.), (.6, -.3, -3.), (0., 2., 0.)),
])
def test_lateral_error_slows_reference_before_combined_hard_limit(
        monkeypatch, target, position, velocity):
    node = Harness(monkeypatch)
    node.trajectory_profile.start_segment((0., 0., -3.), target)
    node.state_px4.local_position = NS(x=position[0], y=position[1], z=position[2])
    (node.state_px4.current_velocity_x, node.state_px4.current_velocity_y,
     node.state_px4.current_velocity_z) = velocity
    assert math.dist(position, (0., 0., -3.)) < node.navigation_config.tracking_error_limit
    # Atraso longitudinal de apenas .3m liberava aceleração antes, ignorando .6m lateral.
    assert node.trajectory._tracking_cap(3.) < math.sqrt(sum(v*v for v in velocity))


@pytest.mark.parametrize('gap', [10., .4, -.1])
def test_clock_jump_cannot_produce_large_reference_step(monkeypatch, gap):
    node = Harness(monkeypatch)
    node.goto((30., 0., -3.))
    for _ in range(200):
        node.tick()
    before = node.trajectory.measured_position
    node.time += gap
    position, *_ = node.tick()
    assert 'Relógio' in node.trajectory.navigation_error
    assert math.dist(before, position) <= .3


def test_executor_delay_within_reaction_budget_keeps_bounded_reference(monkeypatch):
    node = Harness(monkeypatch)
    node.goto((30., 0., -3.))
    for _ in range(200):
        node.tick()
    before = node.trajectory.measured_position
    node.time += .24
    position, *_ = node.tick()
    assert node.trajectory.last_tick_elapsed == pytest.approx(.26)
    assert node.trajectory.navigation_error is None
    assert math.dist(before, position) <= .3
