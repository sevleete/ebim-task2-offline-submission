from __future__ import annotations

from dataclasses import dataclass

_FR3_VEL = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]
_FR3_ACC = [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]
_FR3_JERK = [7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0]


@dataclass
class JointLimits:

    max_velocity: list[float]
    max_acceleration: list[float]
    max_jerk: list[float]

    @property
    def dof(self) -> int:
        return len(self.max_velocity)

    @classmethod
    def fr3(cls, margin: float = 0.6) -> "JointLimits":
        return cls(
            max_velocity=[v * margin for v in _FR3_VEL],
            max_acceleration=[a * margin for a in _FR3_ACC],
            max_jerk=[j * margin for j in _FR3_JERK],
        )

    @classmethod
    def fr3_dual(cls, margin: float = 0.6) -> "JointLimits":
        s = cls.fr3(margin)
        return cls(
            max_velocity=s.max_velocity * 2,
            max_acceleration=s.max_acceleration * 2,
            max_jerk=s.max_jerk * 2,
        )

    @classmethod
    def from_dict(cls, d: dict, margin: float = 1.0) -> "JointLimits":
        return cls(
            max_velocity=[v * margin for v in d["max_velocity"]],
            max_acceleration=[a * margin for a in d["max_acceleration"]],
            max_jerk=[j * margin for j in d["max_jerk"]],
        )
