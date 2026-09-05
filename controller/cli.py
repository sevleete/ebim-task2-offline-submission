from __future__ import annotations

import argparse
import sys

from .hosts import load_config
from . import bringup, control, monitor


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="tmrctl", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--config", default=None,
                   help="hosts.yaml 路径(默认用包内)")
    p.add_argument("--dry-run", action="store_true",
                   help="只打印将执行的远程命令,不下发")

    g = p.add_argument_group("启动编排")
    g.add_argument("--up", choices=["ctl", "teleop"], metavar="{ctl,teleop}",
                   help="拉起一组:ctl=本体(base/zed/lidar/spine/grippers/"
                        "arms/wrist_cams);teleop=GELLO+踏板。"
                        "不提供一把全启(gello 与臂同时起会致臂跳变)")
    g.add_argument("--down", choices=["ctl", "teleop"], metavar="{ctl,teleop}",
                   help="停止一组(ctl / teleop)")
    g.add_argument("--only", default=None,
                   help="逗号分隔的服务子集覆盖组,如 spine,arms")
    g.add_argument("--setup_collision", action="store_true",
                   help="(重)设双臂碰撞/力矩阈值")

    t = p.add_argument_group("示教路径(固定环境的安全转移,替代规划器)")
    t.add_argument("--teach", metavar="NAME",
                   help="示教:手拖臂,回车记路点,q 保存(存 controller/paths/)")
    t.add_argument("--play", metavar="NAME", help="回放:逐路点 PTP")
    t.add_argument("--reverse", action="store_true", help="配合 --play 反向回放")
    t.add_argument("--arm", choices=["left", "right"], default="left",
                   help="配合 --teach:示教哪只臂(默认 left)")

    m = p.add_argument_group("监控")
    m.add_argument("--status", action="store_true", help="一次性状态")
    m.add_argument("--monitor", action="store_true", help="状态面板")
    m.add_argument("--watch", action="store_true", help="配合 --monitor 实时刷新")
    m.add_argument("--interval", type=float, default=5.0, help="刷新间隔秒")

    c = p.add_argument_group("硬件控制")
    c.add_argument("--enable",
                   choices=["all", "base", "car", "spine", "left_arm",
                            "right_arm", "grippers"],
                   help="使能子系统(all=spine+双臂)")
    c.add_argument("--spine", type=float, metavar="H", help="升降柱到 H 米")
    c.add_argument("--left_arm_joint", type=float, nargs=7, metavar="J",
                   help="左臂 7 关节位形(PTP,平滑)")
    c.add_argument("--right_arm_joint", type=float, nargs=7, metavar="J",
                   help="右臂 7 关节位形(PTP,平滑)")
    c.add_argument("--left_gripper", type=float, metavar="V",
                   help="左爪开合 0~1")
    c.add_argument("--right_gripper", type=float, metavar="V",
                   help="右爪开合 0~1")
    c.add_argument("--home", choices=["left", "right"],
                   help="某臂回 ready 位姿")
    c.add_argument("--get_joints", choices=["left", "right", "both"],
                   help="读并打印当前关节(找操作位姿用)")
    c.add_argument("--recover", choices=["left", "right"],
                   help="清错并恢复某臂")
    c.add_argument("--rearm", choices=["left", "right"],
                   help="弹某臂 impedance 控制器(遥操作接管按键;配 --off 先关)")
    c.add_argument("--off", action="store_true",
                   help="配合 --rearm:置 inactive(默认 active)")
    c.add_argument("--hold", choices=["left", "right", "both"],
                   help="冻结当前位姿防漂移(发实测关节到目标口;GELLO未跑时用)")
    c.add_argument("--unhold", choices=["left", "right", "both"],
                   help="停止 hold")
    c.add_argument("--base", action="store_true", help="底盘限时速度微调")
    c.add_argument("--vx", type=float, default=0.0, help="底盘 x 速度")
    c.add_argument("--vy", type=float, default=0.0, help="底盘 y 速度")
    c.add_argument("--wz", type=float, default=0.0, help="底盘 yaw 角速度")
    c.add_argument("--dur", type=float, default=1.0, help="底盘持续秒")
    return p


def _split(only: str | None) -> list[str] | None:
    if not only:
        return None
    return [s.strip() for s in only.split(",") if s.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    only = _split(args.only)
    dr = args.dry_run

    if args.up:
        return bringup.up(cfg, only or cfg.groups.get(args.up, []), dr)
    if args.down:
        return bringup.down(cfg, only or cfg.groups.get(args.down, []), dr)
    if args.setup_collision:
        return 0 if bringup.setup_collision(cfg, dr) else 1
    if args.teach:
        from . import paths
        return paths.teach(cfg, args.teach, args.arm)
    if args.play:
        from . import paths
        return paths.play(cfg, args.play, args.reverse, dr)
    if args.status:
        return monitor.status(cfg)
    if args.monitor:
        return monitor.monitor(cfg, args.watch, args.interval)
    if args.spine is not None:
        return control.spine(cfg, args.spine, dr)
    if args.left_arm_joint:
        return control.arm_joint(cfg, "left", args.left_arm_joint, dr)
    if args.right_arm_joint:
        return control.arm_joint(cfg, "right", args.right_arm_joint, dr)
    if args.left_gripper is not None:
        return control.gripper(cfg, "left", args.left_gripper, dr)
    if args.right_gripper is not None:
        return control.gripper(cfg, "right", args.right_gripper, dr)
    if args.home:
        return control.arm_home(cfg, args.home, dr)
    if args.get_joints:
        return control.get_joints(cfg, args.get_joints, dr)
    if args.recover:
        return control.recover(cfg, args.recover, dr)
    if args.rearm:
        return control.rearm(cfg, args.rearm, args.off, dr)
    if args.hold:
        return control.hold(cfg, args.hold, dr)
    if args.unhold:
        return control.unhold(cfg, args.unhold, dr)
    if args.base:
        return control.base(cfg, args.vx, args.vy, args.wz, args.dur, dr)
    if args.enable:
        return control.enable(cfg, args.enable, dr)

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
