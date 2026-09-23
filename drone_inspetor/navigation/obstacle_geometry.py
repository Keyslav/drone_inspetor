"""Interseção analítica do volume do drone com setores sem observação.

Cada célula do scan conhece uma distância radial, não apenas um ponto amostrado.
O restante da célula é desconhecido. A cobertura do deslocamento é a união do
scan com o disco que o veículo já ocupa; nenhum novo espaço é presumido livre.
"""

import math

TAU = 2.0 * math.pi
EPSILON = 1e-10


def point_entry(x, y, radius):
    """Primeiro centro (s, 0), s>=0, cujo disco toca o ponto (x, y)."""
    if abs(y) > radius:
        return math.inf
    half_chord = math.sqrt(max(0.0, radius * radius - y * y))
    if x + half_chord < 0.0:
        return math.inf
    return max(0.0, x - half_chord)


def _inside(angle, start, end):
    return (angle - start) % TAU <= end - start + EPSILON


def _partitions(start, end, angles):
    values = [start, end]
    for angle in angles:
        value = start + (angle - start) % TAU
        if start + EPSILON < value < end - EPSILON:
            values.append(value)
    values = sorted(set(values))
    return zip(values, values[1:])


def _ray_circle(origin, direction, radius):
    """Parâmetros da reta origin + r*direction que interceptam disco na origem."""
    ox, oy = origin
    vx, vy = direction
    projection = ox * vx + oy * vy
    discriminant = projection ** 2 + radius ** 2 - ox ** 2 - oy ** 2
    if discriminant < -EPSILON:
        return None
    root = math.sqrt(max(0., discriminant))
    return -projection - root, -projection + root


def _outside_intervals(origin, direction, lower, safe_radius):
    roots = _ray_circle(origin, direction, safe_radius)
    if roots is None or roots[1] <= lower:
        return [(lower, math.inf)]
    intervals = []
    if roots[0] > lower:
        intervals.append((lower, roots[0]))
    intervals.append((max(lower, roots[1]), math.inf))
    return intervals


def _ray_entry(origin, angle, lower, upper, radius):
    ox, oy = origin
    vx, vy = math.cos(angle), math.sin(angle)
    result = point_entry(ox + lower * vx, oy + lower * vy, radius)
    if math.isfinite(upper):
        result = min(result, point_entry(ox + upper * vx, oy + upper * vy, radius))
    # O primeiro contato no interior é uma tangência com a reta suporte.
    if abs(vy) > EPSILON:
        cross = oy * vx - ox * vy
        for side in (-radius, radius):
            distance = (side - cross) / vy
            projection = (distance - ox) * vx - oy * vy
            if distance >= 0 and lower - EPSILON <= projection <= upper + EPSILON:
                result = min(result, distance)
    return result


def _circle_intersections(origin, radial, safe_radius):
    """Pontos comuns ao círculo do sensor e ao disco inicial do veículo."""
    ox, oy = origin
    distance = math.hypot(ox, oy)
    if (distance < EPSILON or distance > radial + safe_radius + EPSILON or
            distance < abs(radial - safe_radius) - EPSILON):
        return []
    along = (radial ** 2 - safe_radius ** 2 + distance ** 2) / (2 * distance)
    height = math.sqrt(max(0., radial ** 2 - along ** 2))
    ux, uy = -ox / distance, -oy / distance
    base = (ox + along * ux, oy + along * uy)
    return [(base[0] - height * uy, base[1] + height * ux),
            (base[0] + height * uy, base[1] - height * ux)]


def _arc_entry(origin, radial, start, end, radius):
    """Contato com arco por extremidades ou tangência entre dois círculos."""
    ox, oy = origin
    result = min(point_entry(ox + radial * math.cos(angle),
                             oy + radial * math.sin(angle), radius)
                 for angle in (start, end))
    for center_distance, sign in ((radial + radius, 1.),
                                  (abs(radial - radius), 1. if radial >= radius else -1.)):
        discriminant = center_distance ** 2 - oy ** 2
        if discriminant < -EPSILON:
            continue
        root = math.sqrt(max(0., discriminant))
        for distance in (ox - root, ox + root):
            if distance < 0:
                continue
            if center_distance < EPSILON:
                result = min(result, distance)
                continue
            angle = math.atan2(sign * -oy, sign * (distance - ox))
            if _inside(angle, start, end):
                result = min(result, distance)
    return result


def unknown_sector_entry(origin, start, end, observed, radius, horizon):
    """Limita translação +X quando o disco encontra uma célula desconhecida.

    A região desconhecida é um setor exterior a ``observed`` menos o disco
    inicial. Sua fronteira contém semirretas, arco radial e partes do arco do
    disco inicial. Todas as partes são examinadas analiticamente; não há grade
    de pontos ao longo do corredor. Coordenadas são relativas à posição/rumo.
    """
    if observed >= math.hypot(*origin) + horizon + radius:
        return math.inf
    safe_radius = radius + 1e-7  # Evita tratar contato inicial tangencial como invasão.
    result = math.inf
    critical_safe = []
    ox, oy = origin
    for angle in (start, end):
        direction = (math.cos(angle), math.sin(angle))
        roots = _ray_circle(origin, direction, safe_radius)
        if roots is not None:
            critical_safe.extend(math.atan2(oy + value * direction[1], ox + value * direction[0])
                                 for value in roots if value >= 0)
        for lower, upper in _outside_intervals(origin, direction, observed, safe_radius):
            result = min(result, _ray_entry(origin, angle, lower, upper, radius))
    intersections = _circle_intersections(origin, observed, safe_radius)
    critical_safe.extend(math.atan2(y, x) for x, y in intersections)
    if observed > 0:
        angles = [math.atan2(y - oy, x - ox) for x, y in intersections]
        for lower, upper in _partitions(start, end, angles):
            middle = (lower + upper) / 2
            x, y = ox + observed * math.cos(middle), oy + observed * math.sin(middle)
            if math.hypot(x, y) >= safe_radius - EPSILON:
                result = min(result, _arc_entry(origin, observed, lower, upper, radius))
    # Quando a medição termina dentro do disco inicial, o limite desconhecido
    # passa a ser o próprio disco, somente nos arcos pertencentes à célula.
    for lower, upper in _partitions(0., TAU, critical_safe):
        middle = (lower + upper) / 2
        x, y = safe_radius * math.cos(middle) - ox, safe_radius * math.sin(middle) - oy
        if math.hypot(x, y) >= observed - EPSILON and _inside(math.atan2(y, x), start, end):
            result = min(result, _arc_entry((0., 0.), safe_radius, lower, upper, radius))
    return 0.0 if result < 1e-6 else result


def coverage_sectors(bearings, ranges, resolution):
    """Células angulares completas e lacunas de FOV explicitamente desconhecidas.

    Bordas externas terminam nos feixes extremos; não expandem o campo de visão.
    Entre feixes consecutivos, cada célula ocupa até o ponto médio angular.
    Distâncias iguais adjacentes são agrupadas sem alterar a geometria.
    """
    samples = {}
    for angle, distance in zip(bearings, ranges):
        # Scans de 360° podem repetir o primeiro feixe no último índice.
        key = round(angle % TAU, 12)
        samples[key] = min(samples.get(key, math.inf), distance)
    angles = sorted(samples)
    if not angles:
        return ((0., TAU, 0.),)
    cells = []
    for index, angle in enumerate(angles):
        previous = angles[index - 1] if index else angles[-1] - TAU
        following = angles[index + 1] if index + 1 < len(angles) else angles[0] + TAU
        start = (previous + angle) / 2 if angle - previous <= resolution * 1.001 else angle
        end = (angle + following) / 2 if following - angle <= resolution * 1.001 else angle
        cells.append((start, end, samples[angle]))
    sectors = []
    for index, (start, end, distance) in enumerate(cells):
        previous_end = cells[index - 1][1] if index else cells[-1][1] - TAU
        if start - previous_end > EPSILON:
            sectors.append((previous_end, start, 0.))
        if end - start > EPSILON:
            if sectors and abs(sectors[-1][1] - start) < EPSILON and sectors[-1][2] == distance:
                sectors[-1] = (sectors[-1][0], end, distance)
            else:
                sectors.append((start, end, distance))
    return tuple(sectors)
