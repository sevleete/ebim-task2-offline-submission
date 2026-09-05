from __future__ import annotations

import os
from dataclasses import dataclass

import yaml

from .base import Smoother
from .limits import JointLimits

_DEFAULT_CFG = os.path.join(os.path.dirname(__file__), "config.yaml")


@dataclass
class Config:
    cycle_time: float
    margin: float
    dof: int
    impl: str
    limits: JointLimits
    demo: dict


def load(path: str | None = None) -> Config:
    path = path or _DEFAULT_CFG
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)

    dof = int(c.get("dof", 7))
    margin = float(c.get("margin", 0.6))
    impl = c.get("impl", "ruckig")
    cycle_time = float(c.get("cycle_time", 0.01))

    if c.get("limits"):
        lim = JointLimits.from_dict(c["limits"], margin)
        if dof == 14 and lim.dof == 7:
            lim = JointLimits(lim.max_velocity * 2, lim.max_acceleration * 2,
                              lim.max_jerk * 2)
    else:
        lim = JointLimits.fr3_dual(margin) if dof == 14 \
            else JointLimits.fr3(margin)

    return Config(cycle_time=cycle_time, margin=margin, dof=lim.dof,
                  impl=impl, limits=lim, demo=c.get("demo", {}))


def build(cfg: Config) -> Smoother:
    from . import make_smoother
    return make_smoother(cfg.limits, cfg.cycle_time, cfg.impl)
