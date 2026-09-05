from __future__ import annotations

import argparse
import sys

from . import make_smoother
from .base import finite_diff_limits
from .config import load


def _demo(args) -> int:
    cfg = load(args.config)
    impl = args.impl or cfg.impl
    seconds = args.seconds if args.seconds is not None \
        else float(cfg.demo.get("seconds", 3.0))
    step_rad = float(cfg.demo.get("step_rad", 0.5))
    dof, dt, lim = cfg.dof, cfg.cycle_time, cfg.limits

    sm = make_smoother(lim, dt, impl)
    start = [0.0] * dof
    target = [step_rad] * dof
    steps = int(seconds / dt)
    traj = sm.hold_response(target, start, steps)

    vmax, amax, jmax = finite_diff_limits(traj, dt)
    print(f"实现:{impl}  dof:{dof}  dt:{dt}s  margin:{cfg.margin}  "
          f"步数:{steps}")
    print(f"目标阶跃:{step_rad}rad(裸发=瞬时跳变=无穷速度/加速度)")
    err = max(abs(traj[-1][j] - target[j]) for j in range(dof))
    print(f"到位误差(末拍):{err:.4f} rad")

    tol = 1.05
    ok = True
    print(f"{'关节':<4}{'|v|达到/上限':<22}{'|a|达到/上限':<22}"
          f"{'|jerk|达到/上限':<26}")
    for j in range(dof):
        okv = vmax[j] <= lim.max_velocity[j] * tol
        oka = amax[j] <= lim.max_acceleration[j] * tol
        okj = jmax[j] <= lim.max_jerk[j] * tol
        ok = ok and okv and oka and okj

        def m(good):
            return "✓" if good else "✗"

        print(f"J{j+1:<3}"
              f"{vmax[j]:6.2f}/{lim.max_velocity[j]:6.2f}{m(okv):<9}"
              f"{amax[j]:7.1f}/{lim.max_acceleration[j]:7.1f}{m(oka):<7}"
              f"{jmax[j]:9.0f}/{lim.max_jerk[j]:9.0f}{m(okj)}")
    print("\n结果:" + ("✓ 所有关节的 vel/acc/jerk 都在限值内(不会抱死)"
                       if ok else "✗ 有关节超限,需检查"))
    return 0 if ok else 1


def _info(args) -> int:
    cfg = load(args.config)
    print(f"dof={cfg.dof}  cycle_time={cfg.cycle_time}s  "
          f"margin={cfg.margin}  impl={cfg.impl}")
    print(f"max_velocity     = {cfg.limits.max_velocity}")
    print(f"max_acceleration = {cfg.limits.max_acceleration}")
    print(f"max_jerk         = {cfg.limits.max_jerk}")
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="smoother", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("demo", help="离线自检:阶跃响应验证不超限")
    d.add_argument("--config", default=None, help="config.yaml 路径")
    d.add_argument("--impl", choices=["ruckig", "clamp"], default=None,
                   help="覆盖 yaml 里的 impl")
    d.add_argument("--seconds", type=float, default=None, help="覆盖仿真时长")
    i = sub.add_parser("info", help="打印当前配置/限值")
    i.add_argument("--config", default=None)
    args = p.parse_args(argv)
    return _demo(args) if args.cmd == "demo" else _info(args)


if __name__ == "__main__":
    sys.exit(main())
