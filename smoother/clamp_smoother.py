from __future__ import annotations

from .base import Smoother
from .limits import JointLimits


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class ClampSmoother(Smoother):
    def __init__(self, limits: JointLimits, cycle_time: float = 0.01):
        self.dof = limits.dof
        self.dt = float(cycle_time)
        self.vmax = list(limits.max_velocity)
        self.amax = list(limits.max_acceleration)
        self.jmax = list(limits.max_jerk)
        self.pos = [0.0] * self.dof
        self.velocity = [0.0] * self.dof
        self.acceleration = [0.0] * self.dof

    def reset(self, current_pos, current_vel=None, current_acc=None) -> None:
        self.pos = list(current_pos)
        self.velocity = list(current_vel) if current_vel else [0.0] * self.dof
        self.acceleration = \
            list(current_acc) if current_acc else [0.0] * self.dof

    def step(self, target) -> list[float]:
        dt = self.dt
        for j in range(self.dof):
            v_des = _clamp((target[j] - self.pos[j]) / dt,
                           -self.vmax[j], self.vmax[j])
            a_des = _clamp((v_des - self.velocity[j]) / dt,
                           -self.amax[j], self.amax[j])
            a = _clamp(a_des,
                       self.acceleration[j] - self.jmax[j] * dt,
                       self.acceleration[j] + self.jmax[j] * dt)
            a = _clamp(a, -self.amax[j], self.amax[j])
            v = _clamp(self.velocity[j] + a * dt, -self.vmax[j], self.vmax[j])
            self.acceleration[j] = a
            self.velocity[j] = v
            self.pos[j] += v * dt
        return list(self.pos)
