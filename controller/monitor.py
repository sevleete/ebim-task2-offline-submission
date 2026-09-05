from __future__ import annotations

import re
from concurrent.futures import ThreadPoolExecutor

from rich.console import Console
from rich.table import Table

from .hosts import Config
from . import ssh

_console = Console()


def _host_row(cfg: Config, name: str):
    h = cfg.host(name)
    up = ssh.reachable(h)
    return ("主机", f"{name} ({h.addr})",
            "[green]可达[/]" if up else "[red]不可达[/]")


def _service_row(cfg: Config, sname: str):
    svc = cfg.services[sname]
    h = cfg.host(svc.host)
    cmd = f"pgrep -af {_q(ssh.bracket_pattern(svc.stop))} 2>/dev/null | head -1"
    r = ssh.run(h, cmd, timeout=10, source=False)
    up = bool(r.out.strip())
    return ("服务", f"{sname} @{svc.host}",
            "[green]运行[/]" if up else "[red]停止[/]")


def _topic_row(cfg: Config, entry: dict):
    h = cfg.host(entry["host"])
    name = entry["name"]
    qos = "--qos-reliability best_effort" if entry.get("be") else ""
    r = ssh.run(h, f"timeout 5 ros2 topic hz {name} {qos} 2>/dev/null",
                timeout=9)
    rates = re.findall(r"average rate:\s*([0-9.]+)", r.out)
    if rates:
        hz = float(rates[-1])
        exp = entry.get("hz")
        tag = "green" if (exp is None or hz >= exp * 0.5) else "yellow"
        return ("话题", name, f"[{tag}]{hz:.1f} Hz[/]")
    return ("话题", name, "[red]无数据[/]")


def _controller_row(cfg: Config, entry: dict):
    h = cfg.host(entry["host"])
    r = ssh.run(h, f"ros2 control list_controllers -c {entry['cm']} "
                   f"2>/dev/null",
                timeout=20)
    if not r.ok or not r.out.strip():
        return ("控制器", entry["name"], "[red]查询失败/未起[/]")
    n_active = len(re.findall(r"\bactive\b", r.out))
    n_total = len([ln for ln in r.out.splitlines() if ln.strip()])
    tag = "green" if n_active else "yellow"
    return ("控制器", f"{entry['name']} cm",
            f"[{tag}]{n_active}/{n_total} active[/]")


def _q(s: str) -> str:
    import shlex
    return shlex.quote(s)


def _collect(cfg: Config) -> list[tuple[str, str, str]]:
    tasks = []
    with ThreadPoolExecutor(max_workers=12) as ex:
        for name in cfg.hosts:
            tasks.append(ex.submit(_host_row, cfg, name))
        for sname in cfg.services:
            tasks.append(ex.submit(_service_row, cfg, sname))
        for entry in cfg.monitor_topics:
            tasks.append(ex.submit(_topic_row, cfg, entry))
        for entry in cfg.controller_managers:
            tasks.append(ex.submit(_controller_row, cfg, entry))
        rows = [t.result() for t in tasks]
    order = {"主机": 0, "服务": 1, "控制器": 2, "话题": 3}
    rows.sort(key=lambda r: order.get(r[0], 9))
    return rows


def _render(rows) -> Table:
    t = Table(title="TMR / Mobile FR3 Duo 状态", expand=False)
    t.add_column("类别", style="bold")
    t.add_column("对象")
    t.add_column("状态")
    last = None
    for cat, obj, st in rows:
        if last is not None and cat != last:
            t.add_section()
        t.add_row(cat, obj, st)
        last = cat
    return t


def status(cfg: Config) -> int:
    _console.print(_render(_collect(cfg)))
    return 0


def monitor(cfg: Config, watch: bool = False, interval: float = 5.0) -> int:
    if not watch:
        return status(cfg)
    _console.print("[dim]实时监控(Ctrl+C 退出)[/]")
    try:
        while True:
            rows = _collect(cfg)
            _console.clear()
            _console.print(_render(rows))
            _console.print(f"[dim]每 {interval:.0f}s 刷新 · Ctrl+C 退出[/]")
            import time
            time.sleep(interval)
    except KeyboardInterrupt:
        _console.print("\n已退出监控")
    return 0
