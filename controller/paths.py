from __future__ import annotations

import os
import time

import yaml

from .hosts import Config
from . import control

_PATHS_DIR = os.path.join(os.path.dirname(__file__), "paths")


def _path_file(name: str) -> str:
    os.makedirs(_PATHS_DIR, exist_ok=True)
    safe = "".join(c for c in name if c.isalnum() or c in "-_")
    if safe != name or not safe:
        raise SystemExit(f"路径名只能含字母数字-_:{name!r}")
    return os.path.join(_PATHS_DIR, f"{safe}.yaml")


def teach(cfg: Config, name: str, arm: str = "left") -> int:
    f = _path_file(name)
    if os.path.exists(f):
        print(f"⚠ {f} 已存在,继续将覆盖(Ctrl-C 放弃)")
    wps: list[list[float]] = []
    print(f"示教 [{name}] arm={arm}")
    print("  手拖臂到路点后按回车记录;d+回车 删上一点;q+回车 保存退出")
    print("  建议:起点、终点必记;中间在方向变化/贴近障碍处加点")
    while True:
        try:
            cmd = input(f"[{len(wps)} 点] > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n放弃,未保存")
            return 1
        if cmd == "q":
            break
        if cmd == "d":
            if wps:
                wps.pop()
                print(f"  已删,剩 {len(wps)} 点")
            continue
        read = control._read_measured(cfg, arm)
        if not read:
            print("  ✗ 读不到关节(臂服务在吗?)")
            continue
        _, pos = read
        wps.append([round(float(v), 4) for v in pos])
        print("  ✓ 记录:" + " ".join(f"{v:.3f}" for v in pos))
    if len(wps) < 2:
        print("✗ 至少要 2 个路点,未保存")
        return 1
    with open(f, "w", encoding="utf-8") as fh:
        yaml.safe_dump({"arm": arm, "created": time.strftime("%F %T"),
                        "waypoints": wps}, fh, allow_unicode=True)
    print(f"✓ 已保存 {len(wps)} 点 → {f}")
    print(f"  回放: tmrctl --play {name}   反向: tmrctl --play {name} --reverse")
    return 0


def play(cfg: Config, name: str, reverse: bool = False,
         dry_run: bool = False) -> int:
    f = _path_file(name)
    if not os.path.exists(f):
        raise SystemExit(f"路径不存在:{f}(先 --teach {name})")
    with open(f, encoding="utf-8") as fh:
        d = yaml.safe_load(fh)
    arm, wps = d["arm"], list(d["waypoints"])
    if reverse:
        wps.reverse()
    print(f"回放 [{name}] arm={arm} {len(wps)} 点"
          + ("(反向)" if reverse else ""))
    for i, wp in enumerate(wps, 1):
        print(f"—— 路点 {i}/{len(wps)} ——")
        rc = control.arm_joint(cfg, arm, wp, dry_run=dry_run)
        if rc != 0:
            print(f"✗ 路点 {i} 失败,中止(臂停在当前段终点,安全)")
            return rc
    print("✓ 回放完成")
    return 0
