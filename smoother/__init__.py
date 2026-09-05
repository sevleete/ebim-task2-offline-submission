from __future__ import annotations

from .base import Smoother, finite_diff_limits
from .limits import JointLimits

__all__ = ["Smoother", "JointLimits", "finite_diff_limits", "make_smoother",
           "from_config", "main"]


def make_smoother(limits: JointLimits, cycle_time: float = 0.01,
                  impl: str = "ruckig") -> Smoother:
    if impl == "ruckig":
        from .ruckig_smoother import RuckigSmoother
        return RuckigSmoother(limits, cycle_time)
    if impl == "clamp":
        from .clamp_smoother import ClampSmoother
        return ClampSmoother(limits, cycle_time)
    raise ValueError(f"未知 impl:{impl}")


def from_config(path: str | None = None) -> Smoother:
    from .config import build, load
    return build(load(path))


def main(argv=None):
    from .cli import main as _main
    return _main(argv)
