"""Envelope de frenagem com aceleração contínua e jerk limitado, em SI.

O cálculo Ruckig é local e de estado a estado. Não utiliza intermediate_positions
nem a API remota de waypoints. O autopilot continua responsável pelo controle.
"""

import math

from ruckig import ControlInterface, InputParameter, Ruckig, Trajectory


def positive(value, name):
    """Valida um limite físico antes de entregar o problema ao gerador."""
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} deve ser finito e positivo')
    return float(value)


def braking_distance(speed, acceleration, deceleration, jerk, reaction_time=0.0):
    """Distância até repouso, incluindo latência e aceleração inicial positiva.

    Ignorar a aceleração negativa inicial é conservador: não pressupõe que o
    veículo já tenha atingido a frenagem desejada. Reação mantém aceleração.
    """
    positive(deceleration, 'deceleration')
    positive(jerk, 'jerk')
    if not all(math.isfinite(x) for x in (speed, acceleration, reaction_time)):
        raise ValueError('Estado de frenagem não finito')
    speed = max(0.0, speed)
    acceleration = max(0.0, acceleration)
    reaction_time = max(0.0, reaction_time)
    reaction_distance = speed * reaction_time + acceleration * reaction_time ** 2 / 2
    speed += acceleration * reaction_time
    if speed == 0 and acceleration == 0:
        return 0.0
    request = InputParameter(1)
    request.control_interface = ControlInterface.Velocity
    request.current_position = [0.0]
    request.current_velocity = [speed]
    request.current_acceleration = [acceleration]
    request.target_velocity = [0.0]
    request.target_acceleration = [0.0]
    request.max_acceleration = [max(deceleration, acceleration)]
    request.min_acceleration = [-deceleration]
    request.max_jerk = [jerk]
    trajectory = Trajectory(1)
    result = Ruckig(1).calculate(request, trajectory)
    if result.value < 0:
        raise RuntimeError(f'Falha no envelope de frenagem: {result}')
    position, _, _ = trajectory.at_time(trajectory.duration)
    return reaction_distance + max(0.0, position[0])


def speed_for_clearance(distance, cruise, deceleration, jerk, reaction_time, acceleration=0.0):
    """Maior velocidade cujo envelope de parada cabe no corredor observado."""
    positive(cruise, 'cruise')
    if math.isnan(distance) or distance <= 0:
        return 0.0
    if math.isinf(distance):
        return cruise
    lo, hi = 0.0, cruise
    for _ in range(22):
        speed = (lo + hi) / 2
        if braking_distance(speed, acceleration, deceleration, jerk, reaction_time) <= distance:
            lo = speed
        else:
            hi = speed
    return lo


def terminal_speed_limit(remaining, cruise, deceleration, jerk, reaction_time,
                         approach_velocity, settling_time, acceleration=0.0):
    """Reserva um trecho lento antes da parada final do perfil de posição.

    O piso positivo permite que Ruckig termine no alvo em tempo finito, inclusive
    quando o veículo medido já o ultrapassou. Não deve ser aplicado ao limite de
    obstáculos: um corredor bloqueado continua exigindo velocidade zero.
    O tempo de acomodação é um parâmetro de planejamento, não prova de estabilidade
    dos controladores PX4 nem substituto da validação do envelope físico.
    """
    approach = min(positive(approach_velocity, 'approach_velocity'), cruise)
    positive(settling_time, 'settling_time')
    reserve = approach * settling_time + braking_distance(
        approach, 0., deceleration, jerk, reaction_time)
    return max(approach, speed_for_clearance(
        max(0., remaining - reserve), cruise, deceleration, jerk, reaction_time, acceleration))


class SegmentProfile:
    """Referência escalar S-curve sobre um segmento retilíneo NED.

    Posição, velocidade e aceleração saem da mesma trajetória. Reduções do limite
    replanejam a partir da referência anterior, preservando continuidade. Um limite
    zero freia até repouso sem fingir chegada ao destino. A posição medida só
    confirma chegada; nunca se integra repetidamente uma distância medida defasada.
    """

    def __init__(self, cruise, acceleration, deceleration, jerk,
                 position_tolerance=0.2, velocity_tolerance=0.1):
        self.cruise = positive(cruise, 'cruise')
        self.acceleration = positive(acceleration, 'acceleration')
        self.deceleration = positive(deceleration, 'deceleration')
        self.jerk = positive(jerk, 'jerk')
        self.position_tolerance = positive(position_tolerance, 'position_tolerance')
        self.velocity_tolerance = positive(velocity_tolerance, 'velocity_tolerance')
        self._generator = Ruckig(1)
        self.reset()

    def reset(self):
        self.origin = self.target = self.direction = None
        self.length = self.position = self.velocity = self.accel = 0.0
        self.finished = False
        self.reference_finished = False

    def start(self, origin, target, velocity=0.0, acceleration=0.0):
        if len(origin) != 3 or len(target) != 3:
            raise ValueError('Posições devem ter três componentes NED')
        if not all(math.isfinite(x) for x in (*origin, *target, velocity, acceleration)):
            raise ValueError('Segmento não finito')
        self.reset()
        self.origin, self.target = tuple(origin), tuple(target)
        delta = tuple(b - a for a, b in zip(origin, target))
        self.length = math.sqrt(sum(x * x for x in delta))
        # Ruckig resolve uma coordenada escalar ao longo do segmento. Só ao
        # produzir a saída ela volta aos três eixos NED, mantendo p/v/a coerentes.
        self.direction = tuple(x / self.length for x in delta) if self.length > 1e-9 else (0., 0., 0.)
        self.velocity, self.accel = velocity, acceleration

    @property
    def stopped(self):
        return abs(self.velocity) < 1e-6 and abs(self.accel) < 1e-6

    def step(self, dt, measured_position, speed_limit=None, measured_velocity=None):
        positive(dt, 'dt')
        if self.target is None:
            raise RuntimeError('Segmento não inicializado')
        cap = self.cruise if speed_limit is None else speed_limit
        if not math.isfinite(cap) or cap < 0:
            raise ValueError('Limite de velocidade inválido')
        cap = min(cap, self.cruise)
        request = InputParameter(1)
        request.current_position = [self.position]
        request.current_velocity = [self.velocity]
        request.current_acceleration = [self.accel]
        request.target_position = [self.length]
        request.target_velocity = [0.0]
        request.target_acceleration = [0.0]
        request.max_velocity = [max(cap, 1e-6)]
        request.max_acceleration = [self.acceleration]
        request.min_acceleration = [-self.deceleration]
        request.max_jerk = [self.jerk]
        if cap <= 1e-5:
            # Parar onde a frenagem terminar é diferente de alcançar o waypoint.
            # O modo velocidade evita forçar a chegada quando o corredor fechou.
            request.control_interface = ControlInterface.Velocity
        trajectory = Trajectory(1)
        result = self._generator.calculate(request, trajectory)
        if result.value < 0:
            raise RuntimeError(f'Não foi possível calcular trajetória: {result}')
        position, velocity, acceleration = trajectory.at_time(min(dt, trajectory.duration))
        self.position, self.velocity, self.accel = position[0], velocity[0], acceleration[0]
        self.reference_finished = cap > 1e-5 and dt >= trajectory.duration
        measured_speed = (math.sqrt(sum(x * x for x in measured_velocity))
                          if measured_velocity is not None else abs(self.velocity))
        self.finished = (self.reference_finished
                         and math.dist(measured_position, self.target) <= self.position_tolerance
                         and measured_speed <= self.velocity_tolerance)
        pos = tuple(o + d * self.position for o, d in zip(self.origin, self.direction))
        vel = tuple(d * self.velocity for d in self.direction)
        acc = tuple(d * self.accel for d in self.direction)
        return pos, vel, acc
