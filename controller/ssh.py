from __future__ import annotations

import os
import shlex
import subprocess
from dataclasses import dataclass

_CM_PATH = os.path.expanduser("~/.ssh/cm-tmrctl-%r@%h:%p")
_SSH_OPTS = [
    "-o", "ControlMaster=auto",
    "-o", f"ControlPath={_CM_PATH}",
    "-o", "ControlPersist=120",
    "-o", "ConnectTimeout=8",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "BatchMode=yes",
]


@dataclass
class Host:

    name: str
    addr: str
    user: str
    source: str

    @property
    def target(self) -> str:
        return f"{self.user}@{self.addr}"


@dataclass
class Result:
    rc: int
    out: str
    err: str

    @property
    def ok(self) -> bool:
        return self.rc == 0


def run(
    host: Host,
    remote_cmd: str,
    *,
    timeout: float = 30.0,
    source: bool = True,
) -> Result:
    if source and host.source:
        remote_cmd = f"{host.source} && {remote_cmd}"
    wrapped = f"bash -lc {shlex.quote(remote_cmd)}"
    argv = ["ssh", *_SSH_OPTS, host.target, wrapped]
    try:
        p = subprocess.run(
            argv, capture_output=True, text=True, timeout=timeout
        )
        return Result(p.returncode, p.stdout, p.stderr)
    except subprocess.TimeoutExpired:
        return Result(124, "", f"ssh 超时({timeout}s)")
    except FileNotFoundError:
        return Result(127, "", "本机没有 ssh 客户端")


def reachable(host: Host, timeout: float = 8.0) -> bool:
    r = run(host, "echo ok", timeout=timeout, source=False)
    return r.ok and "ok" in r.out


def bracket_pattern(pat: str) -> str:
    out = []
    for p in pat.split("|"):
        if p and p[0].isalnum():
            out.append(f"[{p[0]}]{p[1:]}")
        else:
            out.append(p)
    return "|".join(out)
