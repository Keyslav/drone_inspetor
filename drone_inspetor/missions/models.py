"""Contratos de missão: posições globais WGS84/AMSL e decolagem relativa ao home."""

import math
from dataclasses import dataclass
from typing import Mapping


class MissionValidationError(ValueError):
    """Indica um campo inválido na definição de missão."""


def _number(data, key, *, default=None, minimum=None, maximum=None):
    value = data.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionValidationError(f'{key}: esperado número finito')
    value = float(value)
    if not math.isfinite(value):
        raise MissionValidationError(f'{key}: esperado número finito')
    if minimum is not None and value < minimum:
        raise MissionValidationError(f'{key}: mínimo {minimum}')
    if maximum is not None and value > maximum:
        raise MissionValidationError(f'{key}: máximo {maximum}')
    return value


def _text(data, key, *, default='', required=False):
    value = data.get(key, default)
    if not isinstance(value, str) or (required and not value.strip()):
        raise MissionValidationError(f'{key}: esperado texto não vazio')
    return value.strip()


def _boolean(data, key, default=False):
    value = data.get(key, default)
    if not isinstance(value, bool):
        raise MissionValidationError(f'{key}: esperado booleano')
    return value


@dataclass(frozen=True)
class InspectionTarget:
    """Objeto e classes de anomalias esperadas em um ponto de inspeção."""

    object_name: str
    anomaly_types: tuple[str, ...] = ()


@dataclass(frozen=True)
class Waypoint:
    """Destino WGS84; altitude global AMSL em metros e yaw em graus."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    yaw_deg: float | None = None
    focus_latitude_deg: float | None = None
    focus_longitude_deg: float | None = None
    inspection: InspectionTarget | None = None
    dwell_s: float | None = None

    @classmethod
    def from_mapping(cls, data):
        """Valida um waypoint no formato JSON público já usado pelo dashboard."""
        if not isinstance(data, Mapping):
            raise MissionValidationError('waypoint: esperado objeto')
        if data.get('command', 'GOTO') != 'GOTO':
            raise MissionValidationError('command: waypoint aceita somente GOTO')
        has_focus = data.get('focus_lat') is not None or data.get('focus_lon') is not None
        use_focus = _boolean(data, 'use_focus', has_focus)
        focus_lat = focus_lon = None
        if has_focus or use_focus:
            focus_lat = _number(data, 'focus_lat', minimum=-90, maximum=90)
            focus_lon = _number(data, 'focus_lon', minimum=-180, maximum=180)
        if not use_focus:
            focus_lat = focus_lon = None
        target = None
        if _boolean(data, 'ponto_de_deteccao'):
            name = _text(data, 'objeto_alvo', required=True)
            anomalies = data.get('tipos_anomalia', [])
            if not isinstance(anomalies, list) or any(
                not isinstance(value, str) or not value.strip() for value in anomalies
            ):
                raise MissionValidationError('tipos_anomalia: esperada lista de nomes')
            target = InspectionTarget(name, tuple(value.strip() for value in anomalies))
        dwell = None
        if 'tempo_de_permanencia' in data:
            dwell = _number(data, 'tempo_de_permanencia', minimum=0)
        return cls(
            latitude_deg=_number(data, 'lat', minimum=-90, maximum=90),
            longitude_deg=_number(data, 'lon', minimum=-180, maximum=180),
            altitude_m=_number(data, 'alt'),
            yaw_deg=_number(data, 'yaw') if data.get('yaw') is not None else None,
            focus_latitude_deg=focus_lat,
            focus_longitude_deg=focus_lon,
            inspection=target,
            dwell_s=dwell,
        )

    def to_mapping(self):
        """Produz uma cópia compatível com consumidores legados do JSON."""
        data = {
            'command': 'GOTO', 'lat': self.latitude_deg, 'lon': self.longitude_deg,
            'alt': self.altitude_m, 'ponto_de_deteccao': self.inspection is not None,
        }
        if self.yaw_deg is not None:
            data['yaw'] = self.yaw_deg
        if self.focus_latitude_deg is not None:
            data.update(use_focus=True, focus_lat=self.focus_latitude_deg,
                        focus_lon=self.focus_longitude_deg)
        if self.inspection is not None:
            data.update(objeto_alvo=self.inspection.object_name,
                        tipos_anomalia=list(self.inspection.anomaly_types))
        if self.dwell_s is not None:
            data['tempo_de_permanencia'] = self.dwell_s
        return data


@dataclass(frozen=True)
class MissionDefinition:
    """Uma missão validada, reutilizável entre sessões sem estado mutável."""

    key: str
    name: str
    waypoints: tuple[Waypoint, ...]
    description: str = ''
    detectable_object: str = ''
    takeoff_altitude_m: float = 20.0
    dwell_s: float = 5.0

    @classmethod
    def from_mapping(cls, key, data, *, takeoff_altitude_m=20.0, dwell_s=5.0):
        """Valida todos os pontos antes de autorizar o início de uma missão."""
        if not isinstance(key, str) or not key.strip() or not isinstance(data, Mapping):
            raise MissionValidationError('missão: esperados identificador e objeto')
        altitude = _number(data, 'takeoff_altitude', default=takeoff_altitude_m, minimum=0.1)
        dwell = _number(data, 'tempo_de_permanencia', default=dwell_s, minimum=0)
        points = data.get('pontos_de_inspecao')
        if not isinstance(points, list) or not points:
            raise MissionValidationError('pontos_de_inspecao: esperada lista não vazia')
        waypoints = []
        for index, point in enumerate(points):
            try:
                waypoints.append(Waypoint.from_mapping(point))
            except MissionValidationError as error:
                raise MissionValidationError(f'ponto {index}: {error}') from error
        return cls(key, _text(data, 'nome', default=key, required=True), tuple(waypoints),
                   _text(data, 'descricao'), _text(data, 'objeto_detectavel'), altitude, dwell)

    def to_mapping(self):
        """Retorna dados independentes no contrato JSON original."""
        return {
            'nome': self.name, 'descricao': self.description,
            'objeto_detectavel': self.detectable_object,
            'takeoff_altitude': self.takeoff_altitude_m,
            'tempo_de_permanencia': self.dwell_s,
            'pontos_de_inspecao': [point.to_mapping() for point in self.waypoints],
        }
