"""Conversões explícitas de eixos, sem ROS, TF ou mudança implícita de origem.

Navegação interna: NED. Sensores horizontais: FLU. Valores PX4 já são NED/FRD;
não aplicar ENU→NED ao receber /fmu/out. Consulte docs/COORDENADAS.md.
"""

import math


def enu_to_ned(vector):
    """(Leste, Norte, Cima) → (Norte, Leste, Abaixo), mesma origem."""
    east, north, up = vector
    return north, east, -up


def ned_to_enu(vector):
    """Rotação inversa, também válida para velocidade e aceleração."""
    return enu_to_ned(vector)


def flu_to_frd(vector):
    """(Frente, Esquerda, Cima) → (Frente, Direita, Abaixo), mesmo corpo."""
    forward, left, up = vector
    return forward, -left, -up


def frd_to_flu(vector):
    """Rotação inversa do corpo; não converte um vetor para o mundo."""
    return flu_to_frd(vector)


def ned_to_legacy_neu(position):
    """Somente posição legada N/L/Cima; NEU não é um referencial destro."""
    north, east, down = position
    return north, east, -down


def yaw_enu_to_ned(yaw_rad):
    """Rumo ENU (zero Leste, anti-horário) → NED (zero Norte, horário)."""
    return (math.pi / 2 - yaw_rad + math.pi) % (2 * math.pi) - math.pi


def yaw_ned_to_enu(yaw_rad):
    """Rumo inverso em radianos; válido para yaw, não para quaternion completo."""
    return yaw_enu_to_ned(yaw_rad)


def body_flu_offset_to_ned(offset, yaw_ned_rad):
    """Offset FLU → NED, assumindo corpo nivelado (somente rotação de yaw)."""
    forward, left, up = offset
    cosine, sine = math.cos(yaw_ned_rad), math.sin(yaw_ned_rad)
    return forward * cosine + left * sine, forward * sine - left * cosine, -up


def scan_angle_to_ned(angle_flu_rad, yaw_ned_rad, mount_yaw_flu_rad=0.):
    """Ângulo de scan CCW relativo ao sensor → rumo NED; montagem aplicada uma vez."""
    bearing = yaw_ned_rad - mount_yaw_flu_rad - angle_flu_rad
    return (bearing + math.pi) % (2 * math.pi) - math.pi


def amsl_to_local_down(altitude_amsl, home_altitude_amsl, home_down):
    """Altitude absoluta → Z NED, incluindo o deslocamento local do HOME."""
    return home_down - (altitude_amsl - home_altitude_amsl)
