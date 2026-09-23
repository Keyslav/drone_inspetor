"""API do perfil de segmento usada pelo adaptador de trajetórias do drone."""

from enum import Enum, auto

from drone_inspetor.navigation.motion import SegmentProfile


class TrajectoryPhase(Enum):
    IDLE = auto()
    ACEL = auto()
    CRUISE = auto()
    BRAKE = auto()
    OBSTACLE_BRAKE = auto()
    BLOCKED = auto()
    DONE = auto()


class TrajectoryProfile(SegmentProfile):
    """Perfil Ruckig com estado legível e compatibilidade com a API do nó."""

    def __init__(self, vc, ad, ao, arrival_tol=0.2, arrival_v_tol=0.1, jerk=2.0):
        super().__init__(vc, ad, ao, jerk, arrival_tol, arrival_v_tol)
        self.phase = TrajectoryPhase.IDLE

    def reset(self):
        super().reset()
        self.phase = TrajectoryPhase.IDLE

    def start_segment(self, origin, target, v_initial=0.0):
        self.start(origin, target, velocity=v_initial)
        self.phase = TrajectoryPhase.ACEL

    def is_done(self):
        return self.finished

    def is_active(self):
        return self.target is not None and not self.finished

    @property
    def v_des(self):
        return self.velocity

    @property
    def a_des(self):
        return self.accel

    def tick(self, dt, current_pos, v_target_obs=None, current_velocity=None):
        output = self.step(dt, current_pos, v_target_obs, current_velocity)
        if self.finished:
            self.phase = TrajectoryPhase.DONE
        elif v_target_obs is not None and v_target_obs <= 1e-5:
            self.phase = TrajectoryPhase.BLOCKED if self.stopped else TrajectoryPhase.OBSTACLE_BRAKE
        elif self.accel < -1e-5:
            self.phase = TrajectoryPhase.BRAKE
        elif self.accel > 1e-5:
            self.phase = TrajectoryPhase.ACEL
        else:
            self.phase = TrajectoryPhase.CRUISE
        return output
