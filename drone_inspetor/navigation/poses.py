"""Histórico limitado para associar aquisição de scan à pose correspondente."""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose:
    stamp: float
    position: tuple
    yaw: float


class PoseHistory:
    """Interpola translação/yaw; não extrapola além da tolerância temporal."""

    def __init__(self, capacity=300):
        self.samples = deque(maxlen=capacity)

    def add(self, stamp, position, yaw):
        if not all(math.isfinite(value) for value in (stamp, *position, yaw)):
            return
        if self.samples and stamp < self.samples[-1].stamp:
            self.samples.clear()
        if self.samples and stamp == self.samples[-1].stamp:
            self.samples.pop()
        self.samples.append(Pose(stamp, tuple(position), yaw))

    def at(self, stamp, max_skew=.05):
        if not self.samples:
            return None
        first, last = self.samples[0], self.samples[-1]
        if stamp <= first.stamp:
            return first if first.stamp - stamp <= max_skew else None
        if stamp >= last.stamp:
            return last if stamp - last.stamp <= max_skew else None
        before = first
        for after in self.samples:
            if after.stamp >= stamp and after.stamp > before.stamp:
                # Uma lacuna grande não fornece uma pose conhecida no intervalo.
                if after.stamp - before.stamp > 2 * max_skew:
                    return None
                fraction = (stamp - before.stamp) / (after.stamp - before.stamp)
                position = tuple(a + fraction * (b - a)
                                 for a, b in zip(before.position, after.position))
                delta = (after.yaw - before.yaw + math.pi) % (2 * math.pi) - math.pi
                return Pose(stamp, position, before.yaw + fraction * delta)
            before = after
        return None
