from __future__ import annotations

import pathlib
import subprocess

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
PIXI = pathlib.Path.home() / ".pixi" / "bin" / "pixi"


def _tmrctl(args: list[str], timeout: float, dry: bool) -> bool:
    cmd = [str(PIXI), "run", "tmrctl", *args]
    if dry:
        print(f"[DRY] tmrctl {' '.join(args)}")
        return True
    print(f"→ tmrctl {' '.join(args)}")
    try:
        r = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True,
                           timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"✗ 超时 {timeout}s")
        return False
    tail = (r.stdout or "").strip()[-300:]
    if tail:
        print(f"  {tail}")
    ok = r.returncode == 0 and "✓" in (r.stdout or "")
    if not ok:
        print(f"✗ rc={r.returncode} {(r.stderr or '').strip()[-200:]}")
    return ok


def run_prepare(cfg_approach: dict, dry: bool = False) -> bool:
    s = cfg_approach["spine"]
    p = cfg_approach["prepare"]
    steps = [
        (["--spine", str(s["position"])], 150,
         "spine 失败？先: tmrctl --up ctl --only spine && tmrctl --enable spine"),
        (["--rearm", "left", "--off"], 60, "关左臂阻抗失败？查 arms 服务"),
        (["--rearm", "right", "--off"], 60, "关右臂阻抗失败？查 arms 服务"),
        (["--left_arm_joint", *(str(v) for v in p["left_arm"])], 120,
         "左臂 PTP 失败？tmrctl --enable left_arm（反射后三部曲），FCI 看 Desk"),
        (["--right_arm_joint", *(str(v) for v in p["right_arm"])], 120,
         "右臂 PTP 失败？tmrctl --enable right_arm"),
    ]
    print(f"== 准备段：spine → {s['position']}m，双臂回 prepare 位 ==")
    for args, to, advice in steps:
        if not _tmrctl(args, to, dry):
            print(f"❌ 准备段中止（{advice}）")
            return False
    print("✅ 准备段完成，允许车动")
    return True
