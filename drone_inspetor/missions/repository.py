"""Leitura e validação de missões; carregar não cria uma sessão ou diretório."""

import json
from pathlib import Path

from drone_inspetor.missions.models import MissionDefinition, MissionValidationError


class MissionRepository:
    """Mantém somente definições validadas e troca o catálogo atomicamente."""

    def __init__(self, path, *, takeoff_altitude_m=20.0, dwell_s=5.0):
        """Configura a origem sem ler arquivos ou criar sessões."""
        self.path = Path(path).expanduser()
        self.takeoff_altitude_m = takeoff_altitude_m
        self.dwell_s = dwell_s
        self._missions = {}

    def load(self):
        """Carrega todo o JSON ou preserva o catálogo anterior em caso de erro."""
        with self.path.open(encoding='utf-8') as source:
            data = json.load(source)
        if not isinstance(data, dict):
            raise MissionValidationError('catálogo: esperado objeto com nomes de missões')
        missions = {}
        for key, value in data.items():
            try:
                missions[key] = MissionDefinition.from_mapping(
                    key, value, takeoff_altitude_m=self.takeoff_altitude_m, dwell_s=self.dwell_s)
            except MissionValidationError as error:
                raise MissionValidationError(f'missão {key!r}: {error}') from error
        self._missions = missions
        return dict(missions)

    def get(self, name):
        """Consulta uma definição sem alterar estado nem criar arquivos."""
        try:
            return self._missions[name]
        except KeyError as error:
            raise MissionValidationError(f'Missão {name!r} não encontrada') from error

    def as_mapping(self):
        """Fornece cópia do catálogo para consumidores do formato JSON legado."""
        return {key: mission.to_mapping() for key, mission in self._missions.items()}
