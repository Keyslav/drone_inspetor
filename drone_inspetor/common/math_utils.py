"""
Utilitários matemáticos para normalização de ângulos e conversão GPS.

Centraliza padrões repetidos no drone_node.py, trajectory.py, obstacles.py
e fsm/context.py. Funções puras, sem dependências de ROS.
"""

import math

from drone_inspetor.common.constants import DroneConstants


# =================================================================================================
# NORMALIZAÇÃO DE YAW
# =================================================================================================

def normalize_yaw_deg(deg: float) -> float:
    """
    Normaliza um ângulo em graus para o intervalo [-180, 180].

    Equivalente ao padrão `((deg + 180) % 360) - 180` mas mais legível.
    """
    return ((deg + 180.0) % 360.0) - 180.0


def yaw_deg_to_0_360(deg: float) -> float:
    """Converte um ângulo em graus para o intervalo [0, 360)."""
    return deg % 360.0


def yaw_diff_shortest(current_deg: float, target_deg: float) -> float:
    """
    Diferença angular pelo caminho mais curto (current - target).
    Resultado normalizado em [-180, 180].

    Positivo: target está à direita (sentido horário) de current.
    Negativo: target está à esquerda (sentido anti-horário) de current.
    """
    return normalize_yaw_deg(current_deg - target_deg)


def yaw_step_toward(current_deg: float, target_deg: float, max_step_deg: float) -> float:
    """
    Calcula o próximo passo incremental de yaw em direção ao target.
    Se a distância restante < max_step_deg, retorna target diretamente.
    Resultado normalizado em [-180, 180].
    """
    diff = yaw_diff_shortest(target_deg, current_deg)  # caminho mais curto
    if abs(diff) <= max_step_deg:
        return normalize_yaw_deg(target_deg)
    step = max_step_deg if diff > 0 else -max_step_deg
    return normalize_yaw_deg(current_deg + step)


# =================================================================================================
# CONVERSÃO GPS ↔ MÉTRICA LOCAL (NED)
# =================================================================================================

def global_to_local_offset(lat1: float, lon1: float, alt1: float,
                            lat2: float, lon2: float, alt2: float) -> tuple:
    """
    Calcula offset NED (norte, leste, altitude) entre duas coordenadas globais.

    Aproximação simples: 1° de latitude ≈ 111132 m. Longitude varia com cos(latitude).
    Suficiente para distâncias < ~10 km em latitudes médias.

    Returns:
        Tuple (dx_norte, dy_leste, dz_altitude) em metros.
    """
    m_per_deg = DroneConstants.METERS_PER_DEGREE
    dx = (lat2 - lat1) * m_per_deg
    dy = (lon2 - lon1) * m_per_deg * math.cos(math.radians(lat1))
    dz = alt2 - alt1
    return (dx, dy, dz)


def horizontal_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Distância euclidiana 2D no plano XY (metros)."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def distance_3d(p1: tuple, p2: tuple) -> float:
    """Distância euclidiana 3D entre dois pontos (x, y, z)."""
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)
