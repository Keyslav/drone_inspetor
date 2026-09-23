"""Limites físicos e margens da navegação, em unidades SI."""

from dataclasses import dataclass, fields
import math

from drone_inspetor.navigation.motion import positive


@dataclass(frozen=True)
class NavigationConfig:
    jerk_limit: float = 2.0
    vertical_velocity: float = 1.0
    yaw_rate_deg: float = 30.0
    yaw_stabilization_seconds: float = 0.5
    vehicle_radius: float = 0.35
    obstacle_margin: float = 0.45
    sensor_timeout: float = 0.75
    telemetry_timeout: float = 0.5
    sensor_pose_max_skew: float = 0.05
    reaction_time: float = 0.35
    planning_distance: float = 6.0
    detour_distance: float = 3.0
    near_obstacle_velocity: float = 0.6
    arrival_approach_velocity: float = 0.6
    arrival_settling_time: float = 2.0
    tracking_error_limit: float = 1.0
    reference_lead_limit: float = 0.5
    tracking_gain: float = 2.0
    blocked_timeout: float = 15.0
    max_detours: int = 12
    lidar_mount_yaw_deg: float = 0.0
    lidar_offset_forward: float = 0.0
    lidar_offset_left: float = 0.0
    depth_offset_forward: float = 0.0
    depth_offset_left: float = 0.0
    down_clearance: float = 0.5

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name == 'lidar_mount_yaw_deg' or '_offset_' in field.name:
                if not math.isfinite(value):
                    raise ValueError('Montagem do lidar deve ser finita')
            else:
                positive(value, field.name)
        if self.planning_distance <= self.vehicle_radius + self.obstacle_margin:
            raise ValueError('Distância de planejamento menor que o volume inflado')
        if self.detour_distance >= self.planning_distance:
            raise ValueError('Desvio deve caber na distância de planejamento')
        if self.reference_lead_limit >= self.tracking_error_limit:
            raise ValueError('Referência deve desacelerar antes do limite de erro')
        if not isinstance(self.max_detours, int):
            raise ValueError('max_detours deve ser inteiro')

    @classmethod
    def from_node(cls, node):
        from drone_inspetor.common.param_utils import load_param
        defaults = cls()
        return cls(**{field.name: load_param(node, f'navigation.{field.name}',
                                            getattr(defaults, field.name))
                      for field in fields(cls)})
