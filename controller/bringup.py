from __future__ import annotations

import json
import shlex
import time

from .hosts import Config, Service
from . import ssh


def _log(msg: str) -> None:
    print(msg, flush=True)


def _filter(cfg: Config, only: list[str] | None) -> list[str]:
    order = cfg.bringup_order
    if not only:
        return list(order)
    want = set(only)
    unknown = want - set(cfg.services)
    if unknown:
        raise SystemExit(f"未知服务:{', '.join(sorted(unknown))}")
    return [s for s in order if s in want]


def _kill_cmd(svc: Service) -> str:
    return (f"pkill -f {shlex.quote(ssh.bracket_pattern(svc.stop))} "
            f">/dev/null 2>&1; sleep 1; true")


def _launch_cmd(cfg: Config, svc: Service) -> str:
    host = cfg.host(svc.host)
    inner = f"{host.source} && {svc.launch}" if host.source else svc.launch
    log = f"~/tmr_logs/{svc.session}.log"
    return (
        f"mkdir -p ~/tmr_logs; "
        f"nohup bash -lc {shlex.quote(inner)} > {log} 2>&1 < /dev/null & "
        f"echo LAUNCHED pid=$!"
    )


def _ready_cmd(svc: Service) -> str | None:
    r = svc.ready or {}
    kind = r.get("type", "none")
    if kind == "none":
        return None
    name = r["name"]
    timeout = int(r.get("timeout", 30))
    listing = "topic list" if kind == "topic" else "service list"
    return (
        f"end=$((SECONDS+{timeout})); "
        f"while [ $SECONDS -lt $end ]; do "
        f"ros2 {listing} 2>/dev/null | grep -qF {shlex.quote(name)} "
        f"&& {{ echo READY; exit 0; }}; sleep 2; done; "
        f"echo TIMEOUT; exit 1"
    )


def _collision_calls(cfg: Config) -> list[tuple[str, str]]:
    payload = json.dumps(cfg.collision_behavior)
    calls = []
    for arm in ("left", "right"):
        srv = cfg.iface(f"{arm}_collision")["service"]
        typ = cfg.iface(f"{arm}_collision")["type"]
        cmd = (
            f"{cfg.collision_source} && "
            f"ros2 service call {srv} {typ} {shlex.quote(payload)}"
        )
        calls.append((arm, cmd))
    return calls


def setup_collision(cfg: Config, dry_run: bool = False) -> bool:
    host = cfg.host("arms")
    ok = True
    for arm, cmd in _collision_calls(cfg):
        if dry_run:
            _log(f"  [dry-run] ({host.name}) {cmd}")
            continue
        _log(f"  → 设 {arm} 臂碰撞阈值 ...")
        r = ssh.run(host, cmd, timeout=60, source=False)
        if r.ok:
            _log(f"  ✓ {arm} 臂阈值已设")
        else:
            ok = False
            _log(f"  ✗ {arm} 臂设置失败:{(r.err or r.out).strip()[:160]}")
    return ok


def up(cfg: Config, only: list[str] | None = None,
       dry_run: bool = False) -> int:
    names = _filter(cfg, only)
    _log(f"启动顺序:{' → '.join(names)}")
    failed: list[str] = []

    for name in names:
        svc = cfg.services[name]
        host = cfg.host(svc.host)
        _log(f"\n[{name}] {svc.desc}  ({host.name} {host.addr})")

        if dry_run:
            _log(f"  [dry-run] {_kill_cmd(svc)}")
            _log(f"  [dry-run] {_launch_cmd(cfg, svc)}")
            if svc.post == "setup_collision":
                setup_collision(cfg, dry_run=True)
            continue

        if not ssh.reachable(host):
            _log(f"  ✗ 主机不可达({host.target}),跳过")
            failed.append(name)
            continue

        ssh.run(host, _kill_cmd(svc), source=False, timeout=10)
        r = ssh.run(host, _launch_cmd(cfg, svc), timeout=20, source=False)
        if not r.ok:
            _log(f"  ✗ 拉起失败:{(r.err or r.out).strip()[:160]}")
            failed.append(name)
            continue
        _log(f"  → 已拉起(日志 ~/tmr_logs/{svc.session}.log)")

        ready = _ready_cmd(svc)
        if ready:
            wait = int(svc.ready.get("timeout", 30))
            _log(f"  … 等就绪(≤{wait}s):{svc.ready['name']}")
            rr = ssh.run(host, ready, timeout=wait + 10)
            if rr.ok:
                _log("  ✓ 就绪")
            else:
                _log("  ✗ 就绪超时(服务可能仍在起,查日志)")
                failed.append(name)

        if svc.post == "setup_collision":
            _log("  → arms 就绪,自动设碰撞阈值")
            setup_collision(cfg, dry_run=False)

    if failed:
        _log(f"\n⚠ 有服务未确认:{', '.join(failed)}")
        return 1
    _log("\n✓ 全部服务已拉起")
    return 0


def down(cfg: Config, only: list[str] | None = None,
         dry_run: bool = False) -> int:
    names = list(reversed(_filter(cfg, only)))
    _log(f"停止顺序:{' → '.join(names)}")
    rc = 0
    for name in names:
        svc = cfg.services[name]
        host = cfg.host(svc.host)
        pat = shlex.quote(ssh.bracket_pattern(svc.stop))
        cmd = (
            f"pkill -f {pat} 2>/dev/null; "
            f"for i in $(seq 8); do "
            f"pgrep -f {pat} >/dev/null || {{ echo STOPPED; exit 0; }}; "
            f"sleep 1; done; "
            f"pkill -9 -f {pat} 2>/dev/null; sleep 1; "
            f"pgrep -f {pat} >/dev/null || {{ echo STOPPED; exit 0; }}; "
            f"pgrep -af {pat} | head -2"
        )
        if dry_run:
            _log(f"  [dry-run] ({host.name}) {cmd}")
            continue
        r = ssh.run(host, cmd, timeout=25, source=False)
        out = r.out.strip()
        if "STOPPED" in out or not out:
            _log(f"  ✓ 已停 {name}")
            continue
        pid = out.split()[0]
        ro = ssh.run(host, f"ps -o user= -p {pid} 2>/dev/null",
                     timeout=8, source=False)
        owner = ro.out.strip()
        if not owner:
            _log(f"  ✓ 已停 {name}(退出较慢)")
            continue
        rc = 1
        _log(f"  ✗ {name} 仍在运行(属主 {owner},{host.user} 的 pkill "
             f"无权杀):{out.splitlines()[0][:100]}")
        if owner != host.user:
            _log("    提示:属主不同,多半是容器/他人进程 —— "
                 "`docker ps` 查容器名后 `docker stop <容器名>`,或找属主处理")
    return rc
