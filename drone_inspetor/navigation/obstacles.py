"""Mapa local de raios e escolha de desvios em coordenadas NED.

LaserScan usa ângulos ROS (anti-horário/FLU). Cada observação é transformada
para NED na aquisição; yaw do veículo e rumo da rota nunca são intercambiados.
Uma região sem observação válida não é considerada livre.
"""

from dataclasses import dataclass
import math

from drone_inspetor.common.coordinates import scan_angle_to_ned

from drone_inspetor.navigation.motion import braking_distance, speed_for_clearance
from drone_inspetor.navigation.obstacle_geometry import (
    coverage_sectors, point_entry, unknown_sector_entry,
)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Scan:
    """Raios horizontais: origem NED, rumos em radianos e stamp monotônico."""

    origin: tuple
    stamp: float
    bearings: tuple
    ranges: tuple
    hits: tuple
    resolution: float
    max_range: float
    sectors: tuple = ()

    def visible(self, point):
        """Verifica se o raio que cobre um ponto alcança a distância requerida."""
        dx, dy = point[0] - self.origin[0], point[1] - self.origin[1]
        distance = math.hypot(dx, dy)
        if distance < 0.05:
            return True
        angle = math.atan2(dy, dx)
        if not self.bearings:
            return False
        if len(self.bearings) == 1:
            index = 0
        else:
            step = wrap(self.bearings[1] - self.bearings[0])
            delta = wrap(angle - self.bearings[0])
            candidates = [round((delta + turns * 2 * math.pi) / step)
                          for turns in (-1, 0, 1)]
            candidates = [index for index in candidates if 0 <= index < len(self.bearings)]
            if not candidates:
                return False
            index = min(candidates, key=lambda i: abs(wrap(self.bearings[i] - angle)))
        return (abs(wrap(self.bearings[index] - angle)) <= self.resolution * 1.6
                and self.ranges[index] >= distance)


class ObstacleMap:
    """Observações por sensor, com expiração e inflação do volume do veículo.

    Retornos de todas as fontes ativas restringem o corredor. Apenas as fontes
    obrigatórias precisam comprovar cobertura livre: por padrão, o LiDAR.
    O depth é complementar e sua ausência não substitui essa cobertura.
    """

    def __init__(self, radius=0.8, timeout=0.75, required_sources=('lidar',)):
        if radius <= 0 or timeout <= 0:
            raise ValueError('Raio e validade de sensores devem ser positivos')
        self.radius, self.timeout = radius, timeout
        self.required_sources = tuple(required_sources)
        self.scans = {}

    def update(self, source, ranges, angle_min, angle_increment, range_min, range_max,
               position, yaw, now, mount_yaw=0.0):
        """Recebe ranges em metros, pose NED e montagem ROS (radianos)."""
        if (len(ranges) == 0 or len(position) < 2 or
                not all(math.isfinite(x) for x in
                        (*position, yaw, now, angle_min, angle_increment,
                         range_min, range_max, mount_yaw)) or
                not 0 < range_min < range_max or angle_increment == 0 or
                abs(angle_increment) > math.pi):
            self.scans.pop(source, None)
            return
        bearings, distances, hits = [], [], []
        for index, distance in enumerate(ranges):
            bearing = scan_angle_to_ned(angle_min + index * angle_increment, yaw, mount_yaw)
            # REP-117: +Inf = sem retorno no alcance; -Inf = próximo demais.
            # NaN/valores fora do contrato não oferecem cobertura livre.
            hit = distance == -math.inf or (math.isfinite(distance) and 0 < distance <= range_max)
            observed = (max(range_min, distance) if hit else
                        range_max if distance == math.inf else 0.0)
            bearings.append(bearing)
            distances.append(observed)
            if hit:
                hits.append((position[0] + observed * math.cos(bearing),
                             position[1] + observed * math.sin(bearing)))
        sectors = coverage_sectors(bearings, distances, abs(angle_increment))
        self.scans[source] = Scan(tuple(position), now, tuple(bearings), tuple(distances),
                                 tuple(hits), abs(angle_increment), range_max, sectors)

    def fresh(self, now):
        return all(source in self.scans and 0 <= now - self.scans[source].stamp <= self.timeout
                   for source in self.required_sources)

    def active_scans(self, now):
        return [scan for scan in self.scans.values() if 0 <= now - scan.stamp <= self.timeout]

    def hit_clearance(self, position, heading, horizon, now):
        """Limite imposto por retornos reais; não comprova cobertura livre."""
        ux, uy = math.cos(heading), math.sin(heading)
        distance = horizon
        for scan in self.active_scans(now):
            for x, y in scan.hits:
                dx, dy = x - position[0], y - position[1]
                distance = min(distance, point_entry(dx * ux + dy * uy,
                                                    dy * ux - dx * uy, self.radius))
        return distance

    def clearance(self, position, heading, horizon, now):
        """Distância livre dentro de uma cápsula inflada na direção de movimento."""
        if not self.fresh(now):
            return 0.0
        ux, uy = math.cos(heading), math.sin(heading)
        distance = self.hit_clearance(position, heading, horizon, now)
        # A região não observada de cada célula angular também limita o volume
        # varrido. Inclui todos os feixes, lacunas do FOV e além de range_max.
        for source in self.required_sources:
            scan = self.scans[source]
            dx, dy = scan.origin[0] - position[0], scan.origin[1] - position[1]
            origin = (dx * ux + dy * uy, dy * ux - dx * uy)
            for start, end, observed in scan.sectors:
                entry = unknown_sector_entry(origin, start - heading, end - heading,
                                             observed, self.radius, distance)
                distance = min(distance, entry)
                if distance <= 0:
                    return 0.0
        return distance

    def nearest(self, position, now):
        return min((math.hypot(x - position[0], y - position[1])
                    for scan in self.active_scans(now) for x, y in scan.hits), default=math.inf)


@dataclass(frozen=True)
class AvoidanceDecision:
    speed_limit: float
    clearance: float
    reason: str
    detour: tuple | None = None


class LocalPlanner:
    """Limita velocidade por frenagem e procura corredor lateral após parar.

    O rumo do desvio persiste durante a manobra. Não há subida arbitrária nem
    alternância esquerda/direita a cada frame. Sem passagem observada, aguarda.
    """

    def __init__(self, cruise=3., braking=2., jerk=2., reaction_time=0.35,
                 planning_distance=6., detour_distance=3., near_speed=0.6):
        self.cruise, self.braking, self.jerk = cruise, braking, jerk
        self.reaction_time = reaction_time
        self.planning_distance, self.detour_distance = planning_distance, detour_distance
        self.near_speed = near_speed
        self.preferred_side = 1

    def evaluate(self, obstacles, position, target, speed, acceleration, now,
                 reference_speed=0., reference_acceleration=0., allow_detour=True):
        if not obstacles.fresh(now):
            return AvoidanceDecision(0., 0., 'sensor_stale')
        dx, dy = target[0] - position[0], target[1] - position[1]
        horizontal = math.hypot(dx, dy)
        if horizontal < 0.1:
            return AvoidanceDecision(self.cruise, math.inf, 'vertical')
        heading = math.atan2(dy, dx)
        horizon = min(horizontal, max(self.planning_distance,
                      braking_distance(max(speed, reference_speed), max(acceleration, reference_acceleration),
                                       self.braking, self.jerk, self.reaction_time) + 2.))
        clearance = obstacles.clearance(position, heading, horizon, now)
        obstructed = clearance < horizon - 0.05
        if obstructed and clearance < self.planning_distance:
            # Troca de direção só após repouso: a nova referência não elimina
            # instantaneamente velocidade/aceleração no ponto de inflexão.
            if max(speed, abs(reference_speed)) > 0.08 or abs(reference_acceleration) > 0.08:
                return AvoidanceDecision(0., clearance, 'braking_for_detour')
            if allow_detour:
                detour = self._detour(obstacles, position, target, heading, now)
                if detour is not None:
                    return AvoidanceDecision(0., clearance, 'detour', detour)
            return AvoidanceDecision(0., clearance, 'blocked')
        # O perfil já freia para o destino. Aplicar o envelope novamente à
        # distância do destino causaria aproximação assintótica em trajetos curtos.
        cap = (speed_for_clearance(clearance, self.cruise, self.braking, self.jerk,
                                   self.reaction_time, max(acceleration, reference_acceleration))
               if obstructed else self.cruise)
        nearest = obstacles.nearest(position, now)
        if nearest < self.planning_distance:
            # near_speed é a ponta lenta desta interpolação de proximidade,
            # não um teto fixo em todo o raio; o envelope de parada pode exigir zero.
            ratio = max(0., min(1., (nearest - obstacles.radius) /
                                (self.planning_distance - obstacles.radius)))
            cap = min(cap, self.near_speed + (self.cruise - self.near_speed) * ratio)
        return AvoidanceDecision(cap, clearance, 'limited' if cap < self.cruise - 0.01 else 'cruise')

    def _detour(self, obstacles, position, target, heading, now):
        candidates = []
        for sign in (self.preferred_side, -self.preferred_side):
            for degrees in (35, 55, 75, 90):
                angle = heading + sign * math.radians(degrees)
                clearance = obstacles.clearance(position, angle, self.detour_distance + 0.2, now)
                if clearance < self.detour_distance:
                    continue
                point = (position[0] + self.detour_distance * math.cos(angle),
                         position[1] + self.detour_distance * math.sin(angle), position[2])
                exit_dx, exit_dy = target[0] - point[0], target[1] - point[1]
                exit_horizon = min(self.planning_distance, math.hypot(exit_dx, exit_dy))
                exit_clearance = obstacles.clearance(
                    point, math.atan2(exit_dy, exit_dx), exit_horizon, now)
                # Primeira perna precisa estar toda observada; a saída só ordena
                # candidatos. Exigir visão do destino oculto impediria contornos.
                # Faixas de 10cm evitam alternar lado por diferenças de um feixe.
                progress = math.cos(math.radians(degrees)) + (0.15 if sign == self.preferred_side else 0)
                candidates.append((round(exit_clearance, 1), progress, sign, point))
        if not candidates:
            return None
        _, _, self.preferred_side, point = max(candidates)
        return point
