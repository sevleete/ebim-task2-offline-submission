from __future__ import annotations

import base64
import json
import shlex

import yaml

from .hosts import Config
from . import ssh
from .robot import ARM_READY, LEFT, N_JOINTS, RIGHT


def _send(cfg: Config, iface_key: str, remote_cmd: str, desc: str,
          *, timeout: float, dry_run: bool) -> int:
    host = cfg.iface_host(iface_key)
    if dry_run:
        print(f"[dry-run] ({host.name} {host.addr}) {remote_cmd}")
        return 0
    print(f"→ {desc}")
    r = ssh.run(host, remote_cmd, timeout=timeout)
    out_all = (r.out or "") + (r.err or "")
    bad = ("success: false" in out_all
           or "status: ABORTED" in out_all
           or "status: CANCELED" in out_all
           or "was rejected" in out_all)
    if r.ok and not bad:
        print(f"✓ {desc} 完成")
        if r.out.strip():
            print(r.out.strip()[:300])
        return 0
    err_lines = [ln for ln in out_all.splitlines()
                 if ("error" in ln.lower() or "success: false" in ln
                     or "ABORTED" in ln) and "type hash" not in ln]
    detail = " | ".join(err_lines[:3]) or (r.err or r.out).strip()[:200]
    print(f"✗ {desc} 失败:{detail[:300]}")
    return 1


def spine(cfg: Config, height: float, dry_run: bool = False) -> int:
    ic = cfg.iface("spine_move")
    goal = {"position": float(height), **cfg.spine_move_params}
    cmd = (f"ros2 action send_goal {ic['action']} {ic['type']} "
           f"'{json.dumps(goal)}'")
    return _send(cfg, "spine_move", cmd, f"spine → {height}m",
                 timeout=60, dry_run=dry_run)


def arm_joint(cfg: Config, side: str, joints: list[float],
              dry_run: bool = False) -> int:
    if len(joints) != N_JOINTS:
        raise SystemExit(f"需要 {N_JOINTS} 个关节值,收到 {len(joints)}")
    key = f"{side}_ptp"
    ic = cfg.iface(key)
    joints_str = "[" + ", ".join(f"{float(j)}" for j in joints) + "]"
    goal = cfg.ptp_goal_template.replace("__JOINTS__", joints_str)
    cmd = f"ros2 action send_goal {ic['action']} {ic['type']} '{goal}'"
    return _send(cfg, key, cmd, f"{side} 臂 PTP → {joints_str}",
                 timeout=60, dry_run=dry_run)


def gripper(cfg: Config, side: str, value: float,
            dry_run: bool = False) -> int:
    value = max(0.0, min(1.0, float(value)))
    key = f"{side}_gripper"
    ic = cfg.iface(key)
    cmd = (f"ros2 topic pub --once {ic['topic']} std_msgs/msg/Float32 "
           f"'{{data: {value}}}'")
    return _send(cfg, key, cmd, f"{side} 爪 → {value}",
                 timeout=20, dry_run=dry_run)


def base(cfg: Config, vx: float, vy: float, wz: float, dur: float,
         dry_run: bool = False) -> int:
    ic = cfg.iface("base_cmd_vel")
    script = (
        "import sys,time,rclpy\n"
        "from rclpy.node import Node\n"
        "from geometry_msgs.msg import TwistStamped\n"
        "tp,vx,vy,wz,dur=sys.argv[1],float(sys.argv[2]),float(sys.argv[3]),"
        "float(sys.argv[4]),float(sys.argv[5])\n"
        "rclpy.init();n=Node('tmr_base_cmd');p=n.create_publisher(TwistStamped,tp,10)\n"
        "def mk(x,y,w):\n"
        " m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg()\n"
        " m.header.frame_id='base_link'\n"
        " m.twist.linear.x=x;m.twist.linear.y=y;m.twist.angular.z=w;return m\n"
        "e=time.time()+dur\n"
        "while time.time()<e:\n"
        " p.publish(mk(vx,vy,wz));time.sleep(0.05)\n"
        "for _ in range(5):\n"
        " p.publish(mk(0.0,0.0,0.0));time.sleep(0.02)\n"
        "n.destroy_node();rclpy.shutdown()\n"
    )
    b64 = base64.b64encode(script.encode()).decode()
    cmd = (f"echo {b64} | base64 -d | python3 - "
           f"{ic['topic']} {float(vx)} {float(vy)} {float(wz)} {float(dur)}")
    return _send(cfg, "base_cmd_vel",
                 cmd, f"底盘 vx={vx} vy={vy} wz={wz} 持续{dur}s",
                 timeout=dur + 20, dry_run=dry_run)


def rearm(cfg: Config, side: str, off: bool = False,
          dry_run: bool = False) -> int:
    host = cfg.host("arms")
    MAX_DELTA = 0.7
    cm0 = f"/{side}/controller_manager"
    if off and not dry_run:
        chk = ssh.run(host, f"ros2 control list_controllers -c {cm0} "
                            f"2>/dev/null | grep joint_impedance_controller",
                      timeout=30)
        st_line = (chk.out or "").strip()
        if not st_line or "inactive" in st_line or "unconfigured" in st_line:
            print(f"✓ {side} impedance controller 已是关闭状态")
            return 0
    if not off and not dry_run:
        read = _read_measured(cfg, side)
        if not read:
            print(f"✗ 拒绝激活:读不到 {side} 臂实测(广播器没起?)")
            return 1
        mnames, mpos = read
        ic = cfg.iface(f"{side}_gello_cmd")
        thost = cfg.iface_host(f"{side}_gello_cmd")
        tgt = _read_topic_joints(thost, ic["topic"])
        if not tgt:
            print(f"✗ 拒绝激活:{ic['topic']} 无发布 —— 先开 gate/relay,"
                  f"目标先行、控制器后开(闸先开,臂后活)")
            return 1
        tnames, tpos = tgt
        m = dict(zip(mnames, mpos))
        deltas = [(n, abs(p - m[n])) for n, p in zip(tnames, tpos) if n in m]
        if not deltas:
            print("✗ 拒绝激活:目标与实测关节名对不上")
            return 1
        worst_name, worst = max(deltas, key=lambda x: x[1])
        if worst > MAX_DELTA:
            print(f"⚠ 目标与实测差较大:{worst_name} 差 {worst:.3f} rad,"
                  f"臂将平滑滑行过去,请让开滑行路径")
        else:
            print(f"✓ 预检通过:目标已在发布,与实测最大差 {worst:.3f} rad"
                  f"({worst_name})")
    st = "inactive" if off else "active"
    cm = f"/{side}/controller_manager"
    if not off:
        if dry_run:
            print(f"[dry-run] ({host.name}) reload+activate impedance {side}")
            return 0
        print(f"→ 重建 {side} impedance controller(恢复平滑滑行)")
        for c in (f"ros2 control set_controller_state -c {cm} "
                  f"joint_impedance_controller inactive",
                  f"ros2 control unload_controller -c {cm} "
                  f"joint_impedance_controller",
                  f"ros2 control load_controller -c {cm} "
                  f"joint_impedance_controller",
                  f"ros2 control set_controller_state -c {cm} "
                  f"joint_impedance_controller inactive"):
            ssh.run(host, c, timeout=40)
    cmd = (f"ros2 control set_controller_state -c {cm} "
           f"joint_impedance_controller {st}")
    desc = f"{side} impedance controller → {st}"
    if dry_run:
        print(f"[dry-run] ({host.name}) {cmd}")
        return 0
    print(f"→ {desc}")
    r = ssh.run(host, cmd, timeout=45)
    out = (r.out or "") + (r.err or "")
    ok = "Successfully" in out or "successfully" in out
    if not ok:
        chk = ssh.run(host, f"ros2 control list_controllers "
                            f"-c /{side}/controller_manager 2>/dev/null "
                            f"| grep joint_impedance_controller", timeout=30)
        ok = st in (chk.out or "")
    print(f"✓ {desc}" if ok else f"✗ {desc}:{out.strip()[:200]}")
    return 0 if ok else 1


def recover(cfg: Config, side: str, dry_run: bool = False) -> int:
    key = f"{side}_recover"
    ic = cfg.iface(key)
    cmd = f"ros2 action send_goal {ic['action']} {ic['type']} '{{}}'"
    return _send(cfg, key, cmd, f"恢复 {side} 臂",
                 timeout=30, dry_run=dry_run)


def enable(cfg: Config, comp: str, dry_run: bool = False) -> int:
    if comp == "all":
        rc = 0
        for c in ("spine", "left_arm", "right_arm", "base", "grippers"):
            rc |= enable(cfg, c, dry_run)
        return rc
    if comp == "spine":
        ic = cfg.iface("spine_on")
        cmd = f"ros2 service call {ic['service']} {ic['type']} '{{}}'"
        return _send(cfg, "spine_on", cmd, "spine 上电",
                     timeout=20, dry_run=dry_run)
    if comp in ("left_arm", "right_arm"):
        side = comp.split("_")[0]
        host = cfg.host("arms")
        if dry_run:
            print(f"[dry-run] ({host.name}) {side} 臂恢复三部曲")
            return 0
        recover(cfg, side)
        print(f"→ ② 激活 {side} 臂硬件接口")
        r = ssh.run(host, f"ros2 control set_hardware_component_state "
                          f"-c /{side}/controller_manager "
                          f"{side}_FrankaHardwareInterface active", timeout=45)
        if "label='active'" not in (r.out or "") and "active" not in (r.out or ""):
            print(f"✗ {side} 硬件激活失败(FCI 被关/急停?看 Desk):"
                  f"{(r.err or r.out).strip()[:120]}")
            return 1
        print(f"→ ③ 激活 {side} 臂状态广播器")
        for c in ("joint_state_broadcaster", "franka_robot_state_broadcaster"):
            ssh.run(host, f"ros2 control set_controller_state "
                          f"-c /{side}/controller_manager {c} active",
                    timeout=35)
        ok = bool(_read_measured(cfg, side))
        print(f"✓ {side} 臂已恢复(measured 在流)" if ok
              else f"✗ {side} measured 仍无数据,广播器没起来")
        return 0 if ok else 1
    if comp in ("base", "car"):
        host = cfg.host("base")
        if dry_run:
            print(f"[dry-run] ({host.name}) 底盘恢复三部曲:recovery→硬件active→弹controller")
            return 0
        print("→ ① 底盘错误恢复")
        ssh.run(host, "timeout 25 ros2 action send_goal "
                      "/action_server/error_recovery "
                      "franka_msgs/action/ErrorRecovery '{}'", timeout=35)
        print("→ ② 激活底盘硬件接口 TmrHardware")
        r = ssh.run(host, "ros2 control set_hardware_component_state "
                          "TmrHardware active", timeout=45)
        if not (r.ok and "active" in (r.out or "")):
            print(f"✗ 硬件激活失败:{(r.err or r.out).strip()[:160]}")
            return 1
        print("→ ③ 弹 swerve 控制器(重新认领接口)")
        ssh.run(host, "ros2 control set_controller_state "
                      "swerve_drive_controller inactive", timeout=35)
        r = ssh.run(host, "ros2 control set_controller_state "
                          "swerve_drive_controller active", timeout=35)
        ok = r.ok and "uccessfully" in ((r.out or "") + (r.err or ""))
        print("✓ 底盘已就绪" if ok
              else f"✗ 控制器激活失败:{(r.err or r.out).strip()[:160]}")
        return 0 if ok else 1
    if comp == "grippers":
        host = cfg.host("arms")
        rc = 0
        for side in ("left", "right"):
            svc = f"/{side}/gripper/robotiq_activation_controller/reactivate_gripper"
            if dry_run:
                print(f"[dry-run] ({host.name}) ros2 service call {svc} ...")
                continue
            print(f"→ 重新激活 {side} 夹爪")
            r = ssh.run(host, f"ros2 service call {svc} std_srvs/srv/Trigger "
                              f"'{{}}'", timeout=30)
            ok = r.ok and "success=True" in (r.out or "")
            print(f"✓ {side} 夹爪已激活" if ok
                  else f"✗ {side} 夹爪激活失败:{(r.err or r.out).strip()[:120]}")
            rc |= 0 if ok else 1
        return rc
    raise SystemExit(f"未知子系统:{comp}"
                     f"(可选 spine/left_arm/right_arm/grippers/base)")


def arm_home(cfg: Config, side: str, dry_run: bool = False) -> int:
    return arm_joint(cfg, side, list(ARM_READY), dry_run=dry_run)


def get_joints(cfg: Config, side: str, dry_run: bool = False) -> int:
    if side == "both":
        rc = 0
        for s in ("left", "right"):
            rc |= get_joints(cfg, s, dry_run)
        return rc
    read = _read_measured(cfg, side)
    if not read:
        print(f"✗ 读不到 {side} 臂关节(机器人未起?)")
        return 1
    _, pos = read
    vals = " ".join(f"{v:.4f}" for v in pos)
    print(f"{side} 当前关节({len(pos)} 轴):")
    print(f"  {vals}")
    print(f"  → tmrctl --{side}_arm_joint {vals}")
    return 0


def _read_topic_joints(host, topic: str):
    r = ssh.run(host, f"timeout 6 ros2 topic echo --once {topic} 2>/dev/null",
                timeout=15)
    if not r.ok or not r.out.strip():
        return None
    try:
        docs = [x for x in yaml.safe_load_all(r.out) if x]
        d = docs[0]
        names = [str(n) for n in d["name"]]
        pos = [float(x) for x in d["position"]]
    except Exception:
        return None
    return (names, pos) if names and len(names) == len(pos) else None


def _read_measured(cfg: Config, side: str):
    ic = cfg.iface(f"{side}_measured")
    host = cfg.iface_host(f"{side}_measured")
    cmd = f"ros2 topic echo {ic['topic']} --once 2>/dev/null"
    r = ssh.run(host, cmd, timeout=12)
    if not r.ok or not r.out.strip():
        return None
    try:
        docs = [x for x in yaml.safe_load_all(r.out) if x]
        d = docs[0]
        names = [str(n) for n in d["name"]]
        pos = [float(x) for x in d["position"]]
    except Exception:
        return None
    if names and pos and len(names) == len(pos):
        return names, pos
    return None


def hold(cfg: Config, side: str, dry_run: bool = False) -> int:
    if side == "both":
        rc = 0
        for s in ("left", "right"):
            rc |= hold(cfg, s, dry_run)
        return rc

    ic = cfg.iface(f"{side}_gello_cmd")
    host = cfg.iface_host(f"{side}_gello_cmd")

    read = _read_measured(cfg, side)
    if read:
        names, pos = read
        names_str = "[" + ", ".join(names) + "]"
        pos_str = "[" + ", ".join(f"{v:.5f}" for v in pos) + "]"
    else:
        fallback = LEFT if side == "left" else RIGHT
        names_str = "[" + ", ".join(fallback) + "]"
        pos_str = "[<read_failed_robot_up?>]"

    msg = f"{{name: {names_str}, position: {pos_str}}}"
    pub = (f"ros2 topic pub -r {cfg.hold_rate} {ic['topic']} {ic['type']} "
           f"'{msg}'")
    log = f"~/tmr_logs/hold_{side}.log"
    inner = f"{host.source} && {pub}" if host.source else pub
    kill = (f"pkill -f "
            f"{shlex.quote(ssh.bracket_pattern('topic pub.*' + ic['topic']))} "
            f">/dev/null 2>&1; sleep 0.3; true")
    remote = (
        f"mkdir -p ~/tmr_logs; "
        f"nohup bash -lc {shlex.quote(inner)} > {log} 2>&1 < /dev/null & "
        f"echo HELD"
    )
    if dry_run or not read:
        tag = "dry-run" if dry_run else "read_failed_print_only"
        print(f"[{tag}] ({host.name}) {kill}")
        print(f"[{tag}] ({host.name}) {remote}")
        return 0 if dry_run else 1
    ssh.run(host, kill, source=False, timeout=10)
    r = ssh.run(host, remote, source=False, timeout=15)
    if r.ok:
        print(f"✓ {side} 臂已 hold(冻结当前位姿,{cfg.hold_rate}Hz)")
        return 0
    print(f"✗ {side} hold 失败:{(r.err or r.out).strip()[:160]}")
    return 1


def unhold(cfg: Config, side: str, dry_run: bool = False) -> int:
    if side == "both":
        rc = 0
        for s in ("left", "right"):
            rc |= unhold(cfg, s, dry_run)
        return rc
    ic = cfg.iface(f"{side}_gello_cmd")
    host = cfg.iface_host(f"{side}_gello_cmd")
    pat = "topic pub.*" + ic["topic"]
    cmd = f"pkill -f {shlex.quote(ssh.bracket_pattern(pat))}"
    if dry_run:
        print(f"[dry-run] ({host.name}) {cmd}")
        return 0
    r = ssh.run(host, cmd, source=False, timeout=10)
    print(f"✓ {side} 臂已取消 hold" if r.rc in (0, 1)
          else f"✗ {side} unhold 失败:{(r.err or r.out).strip()[:120]}")
    return 0
