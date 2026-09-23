"""Conversões numéricas e formatação de telemetria para exibição."""

from datetime import datetime
import math


def quaternion_to_euler(q):
    """Converte quaternion [w, x, y, z] em yaw em graus, no intervalo [0, 360)."""
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360


def calculate_distance(lat1, lon1, lat2, lon2):
    """Retorna distância Haversine em metros para coordenadas em graus."""
    lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    longitude_term = math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
    haversine = math.sin(delta_lat / 2) ** 2 + longitude_term
    # Arredondamento pode ultrapassar [0, 1] em pontos coincidentes/antípodas.
    haversine = max(0.0, min(1.0, haversine))
    return 6371000 * 2 * math.atan2(math.sqrt(haversine), math.sqrt(1 - haversine))


def format_timestamp():
    """Retorna a hora local de apresentação, sem uso em controle ou timeout."""
    return datetime.now().strftime('%H:%M:%S')


def format_coordinates(lat, lon, alt=None):
    """Formata coordenadas em graus e altitude opcional em metros."""
    coordinates = f'{lat:.6f}°, {lon:.6f}°'
    return coordinates if alt is None else f'{coordinates}, {alt:.1f}m'
