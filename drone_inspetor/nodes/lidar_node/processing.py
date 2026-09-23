"""Parsing e classificação legada do LiDAR, em coordenadas ROS FLU."""

import math

import numpy as np


DISTANCE_LIMITS = (8, 5, 3, 2, 1)
SECTORS = ('front', 'left', 'back', 'right')


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Extrai retornos métricos sem inferir ângulos por angle_max arredondado.

    NaN/Inf não são pontos. Leituras positivas saturadas abaixo do mínimo são
    conservadas no mínimo; valores não positivos permanecem desconhecidos.
    """
    values = np.asarray(ranges, dtype=float)
    if (values.ndim != 1 or not all(math.isfinite(value) for value in
            (angle_min, angle_increment, range_min, range_max)) or
            not 0 < range_min < range_max or (values.size > 1 and angle_increment == 0)):
        raise ValueError('Geometria ou limites inválidos no LaserScan')
    angles = angle_min + np.arange(values.size) * angle_increment
    valid = np.isfinite(values) & (values > 0) & (values <= range_max)
    return np.maximum(values[valid], range_min), angles[valid]


def sector_flags(ranges, angles):
    """Snapshot de proximidade: +90° é esquerda, -90° é direita (FLU)."""
    distances = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    if distances.shape != angles.shape:
        raise ValueError('Distâncias e ângulos precisam ter mesmo tamanho')
    valid = np.isfinite(distances) & (distances > 0) & np.isfinite(angles)
    distances, angles = distances[valid], angles[valid]
    flags = {f'have_obstacles_{limit}m': bool(np.any(distances <= limit))
             for limit in DISTANCE_LIMITS}
    flags.update({f'have_obstacles_{sector}_90': False for sector in SECTORS})
    # Cada limite pertence a um único setor; não há duplicação em ±45/±135°.
    quadrants = np.floor(((angles + math.pi / 4) % (2 * math.pi)) / (math.pi / 2)).astype(int)
    for quadrant, name in enumerate(SECTORS):
        flags[f'have_obstacles_{name}_90'] = bool(np.any((distances <= 1) & (quadrants == quadrant)))
    return flags


def ground_distance(ranges, range_min, range_max):
    """Retorna NaN quando o sensor inferior não mediu o chão neste frame."""
    distances, _ = scan_points(ranges, 0., 1., range_min, range_max)
    return float(distances.min()) if distances.size else math.nan


def point_vector(ranges, angles):
    """Mantém o contrato [distância, ângulo, ...] do dashboard."""
    return np.column_stack((ranges, angles)).ravel().tolist()
