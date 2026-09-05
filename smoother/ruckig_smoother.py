from __future__ import annotations

from .base import Smoother
from .limits import JointLimits


class RuckigSmoother(Smoother):
    def __init__(self, limits: JointLimits, cycle_time: float = 0.01):
        from ruckig import InputParameter, OutputParameter, Ruckig

        self.dof = limits.dof
        self.cycle_time = float(cycle_time)
        self._otg = Ruckig(self.dof, self.cycle_time)
        self._inp = InputParameter(self.dof)
        self._out = OutputParameter(self.dof)
        self._inp.max_velocity = list(limits.max_velocity)
        self._inp.max_acceleration = list(limits.max_acceleration)
        self._inp.max_jerk = list(limits.max_jerk)
        self.velocity = [0.0] * self.dof
        self.acceleration = [0.0] * self.dof
        self._ready = False

    def reset(self, current_pos, current_vel=None, current_acc=None) -> None:
        z = [0.0] * self.dof
        self._inp.current_position = list(current_pos)
        self._inp.current_velocity = list(current_vel) if current_vel else z
        self._inp.current_acceleration = \
            list(current_acc) if current_acc else list(z)
        self._inp.target_position = list(current_pos)
        self._inp.target_velocity = list(z)
        self._inp.target_acceleration = list(z)
        self.velocity = list(self._inp.current_velocity)
        self.acceleration = list(self._inp.current_acceleration)
        self._ready = True

    def step(self, target) -> list[float]:
        from ruckig import Result

        if not self._ready:
            raise RuntimeError("RuckigSmoother 未 reset(),请先用当前关节位初始化")
        self._inp.target_position = list(target)
        res = self._otg.update(self._inp, self._out)
        if res not in (Result.Working, Result.Finished):
            raise RuntimeError(f"Ruckig 生成失败:{res}(目标超限或状态非法?)")
        self._out.pass_to_input(self._inp)
        self.velocity = list(self._out.new_velocity)
        self.acceleration = list(self._out.new_acceleration)
        return list(self._out.new_position)
