"""Configuração validada de missão, carregada uma vez na borda ROS."""

import math
from dataclasses import dataclass, fields
from pathlib import Path


@dataclass(frozen=True)
class MissionConfig:
    """Prazos operacionais usam relógio monotônico; permanência usa tempo ROS."""

    missions_file: str = 'missions.json'
    missions_directory: str = '~/Drone_Inspetor_Missoes'
    takeoff_altitude: float = 20.0
    tempo_de_permanencia: float = 5.0
    mission_period: float = 0.6
    topic_health_timeout: float = 5.0
    action_feedback_timeout: float = 60.0
    action_cancel_timeout: float = 5.0
    return_acceptance_timeout: float = 40.0
    return_retry_interval: float = 1.0
    detection_timeout: float = 10.0
    detection_service_timeout: float = 2.0
    cv_control_timeout: float = 5.0

    def __post_init__(self):
        """Rejeita caminhos vazios e prazos que impedem supervisão."""
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, str):
                if not value.strip():
                    raise ValueError(f'{field.name}: caminho não pode ser vazio')
            elif not math.isfinite(value) or value < 0 or (
                field.name != 'tempo_de_permanencia' and value == 0
            ):
                raise ValueError(f'{field.name}: valor inválido {value}')
        if self.detection_timeout < self.detection_service_timeout:
            raise ValueError('detection_timeout deve cobrir detection_service_timeout')

    @classmethod
    def from_node(cls, node):
        """Respeita os parâmetros globais e específicos fornecidos pelo launch."""
        from drone_inspetor.common.param_utils import load_param
        defaults = cls()
        return cls(**{field.name: load_param(node, field.name, getattr(defaults, field.name))
                      for field in fields(cls)})

    def resolve_missions_file(self, package_share_directory):
        """Resolve caminho absoluto ou relativo à pasta missions instalada."""
        path = Path(self.missions_file).expanduser()
        return path if path.is_absolute() else Path(package_share_directory) / 'missions' / path
