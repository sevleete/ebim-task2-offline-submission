from __future__ import annotations

from abc import ABC, abstractmethod


class Smoother(ABC):

    velocity: list[float]
    acceleration: list[float]

    @abstractmethod
    def reset(self, current_pos, current_vel=None, current_acc=None) -> None:
        pass

    @abstractmethod
    def step(self, target) -> list[float]:
        pass

    def smooth(self, targets, current_pos) -> list[list[float]]:
        self.reset(current_pos)
        return [self.step(t) for t in targets]

    def hold_response(self, target, current_pos, steps: int) -> list[list[float]]:
        return self.smooth([target] * steps, current_pos)


def finite_diff_limits(positions, dt: float):
    T = len(positions)
    dof = len(positions[0]) if T else 0

    def col(seq, j):
        return [row[j] for row in seq]

    def diff(seq):
        return [(seq[i + 1] - seq[i]) / dt for i in range(len(seq) - 1)]

    vmax = [0.0] * dof
    amax = [0.0] * dof
    jmax = [0.0] * dof
    for j in range(dof):
        p = col(positions, j)
        v = diff(p)
        a = diff(v)
        jk = diff(a)
        vmax[j] = max((abs(x) for x in v), default=0.0)
        amax[j] = max((abs(x) for x in a), default=0.0)
        jmax[j] = max((abs(x) for x in jk), default=0.0)
    return vmax, amax, jmax
