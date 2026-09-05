from __future__ import annotations

import os
from dataclasses import dataclass, field

import yaml

from .ssh import Host

_DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "hosts.yaml")


@dataclass
class Service:

    name: str
    desc: str
    host: str
    launch: str
    session: str
    stop: str
    ready: dict = field(default_factory=lambda: {"type": "none"})
    post: str | None = None


@dataclass
class Config:
    hosts: dict[str, Host]
    services: dict[str, Service]
    bringup_order: list[str]
    groups: dict[str, list[str]]
    interfaces: dict[str, dict]
    spine_move_params: dict
    ptp_goal_template: str
    collision_source: str
    collision_behavior: dict
    hold_rate: int
    graph_host: str
    monitor_topics: list[dict]
    controller_managers: list[dict]

    def host(self, key: str) -> Host:
        return self.hosts[key]

    def iface(self, key: str) -> dict:
        return self.interfaces[key]

    def iface_host(self, key: str) -> Host:
        return self.hosts[self.interfaces[key]["host"]]


def load_config(path: str | None = None) -> Config:
    path = path or _DEFAULT_YAML
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f)

    hosts = {
        name: Host(name=name, addr=h["addr"], user=h["user"],
                   source=h.get("source", ""))
        for name, h in d["hosts"].items()
    }
    services = {
        name: Service(
            name=name, desc=s.get("desc", name), host=s["host"],
            launch=" ".join(s["launch"].split()),
            session=s["session"], stop=s["stop"],
            ready=s.get("ready", {"type": "none"}), post=s.get("post"),
        )
        for name, s in d["services"].items()
    }
    return Config(
        hosts=hosts,
        services=services,
        bringup_order=d["bringup_order"],
        groups=d.get("groups", {}),
        interfaces=d["interfaces"],
        spine_move_params=d["spine_move_params"],
        ptp_goal_template=d["ptp_goal_template"],
        collision_source=d["collision_source"],
        collision_behavior=d["collision_behavior"],
        hold_rate=int(d.get("hold_rate", 50)),
        graph_host=d["graph_host"],
        monitor_topics=d["monitor_topics"],
        controller_managers=d["controller_managers"],
    )
