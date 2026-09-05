# EBiM 线下赛 Task2 —— 评测指南

> **English:** see [README.md](README.md) · 本文为中文版

Task 2 真机评测程序(Mobile FR3 Duo,导热垫贴装)。这是**操作员在机器人侧主机上要跑的东西**
(双臂主机 172.16.0.100,或任一接入机器人局域网的主机)。

模型推理在参赛方保持在线的服务器上运行,**操作员无需搭建或管理它**。参赛方会在评测前于
issue 评论区给出服务器地址(`SERVER_HOST:SERVER_PORT`),操作员按 §3 用环境变量传入即可。

> **运行时网络**:与"容器不联网"的通用假设不同,本客户端**需要出站网络**去连接该推理服务器
> (评测现场已确认可出外网),除此之外不访问互联网。

---

## 1. 准备(运行主机上,一次性)

前置:Docker Engine + 对三台机器人主机(172.16.0.50 / .100 / .101)免密的 SSH 密钥
(在 .100 上运行时含对自身免密)。

```bash
docker build -t ebim-task2-offline:submission .
docker run --rm ebim-task2-offline:submission health   # 期望输出 "health: PASS"
```

## 2. 硬件启动

1. 机器人总电与两臂控制柜上电,**两个急停全部旋开**;
2. 浏览器打开左臂 Desk(https://172.16.16.12)与右臂 Desk(https://172.16.16.11):
   解锁关节 → **Activate FCI**(两臂各一次);
3. 一键拉起并使能全部硬件(逐个启动+自动重试,最后自查三路相机,看到 ✓✓ 即就绪):

```bash
docker run --rm -it --network host --ipc host \
  -v $HOME/.ssh:/root/.ssh:ro \
  ebim-task2-offline:submission bringup
```

## 3. 运行评测

评测开始时车在初始位置。推理有两种方式;**客户端命令完全一致,只有 `SERVER_HOST` 不同。**

### 3.1 使用远程服务器推理(默认)

参赛方保持推理服务器在线,并在评测前于 issue 评论区给出地址。启动(含车辆自动趋近就位):

```bash
docker run --rm -it --network host --ipc host \
  -v $HOME/.ssh:/root/.ssh:ro \
  -e ROS_DOMAIN_ID=0 \
  -e SERVER_HOST=<参赛方给出的地址> -e SERVER_PORT=6060 \
  ebim-task2-offline:submission run
```

### 3.2 使用本地 NVIDIA GPU 推理(现场配有时)

前提:NVIDIA GPU(显存 ≥12GB)+ NVIDIA Container Toolkit,以及参赛方的模型权重
(提交表单 supplementary 里的 HuggingFace 链接;私有仓,访问可经组委会协调)已下载到本地目录。

```bash
# 1) 构建推理服务器镜像(拉取 torch cu128 + lerobot,数 GB)
docker build -f Dockerfile.server -t ebim-task2-server:submission .

# 2) 挂载权重启动服务器
docker run -d --gpus all -p 6060:6060 \
  -v /path/to/checkpoint:/models/ckpt:ro \
  ebim-task2-server:submission
# 等 `docker logs` 出现「监听 :6060」(首次加载 1~4 分钟)

# 3) 跑客户端 —— 与 3.1 同一条命令,SERVER_HOST=127.0.0.1
docker run --rm -it --network host --ipc host \
  -v $HOME/.ssh:/root/.ssh:ro \
  -e ROS_DOMAIN_ID=0 \
  -e SERVER_HOST=127.0.0.1 -e SERVER_PORT=6060 \
  ebim-task2-offline:submission run
```

### 运行中与回合结束(两种方式通用)

流程**全自动**:车辆趋近并停到桌前 → 双臂/升降柱到起始位形 → 右臂锁定 → 接管左臂 →
自动开始执行策略,无需任何按键。

运行中可用按键:
| 键 | 作用 |
|---|---|
| `p` | 暂停(臂冻结保持,按回车继续) |
| `q` | 结束本回合并退出 |
| Ctrl-C | 急停(停发指令并关闭左臂控制,臂原地停住) |

**回合结束**:放置完成(夹爪持续松开约 5s)后程序**自动结束**并停发指令;也可随时按 `q`
手动结束。之后复位场景(导热垫放回起始位),开始下一回合:
- 车回到初始位置的:重新执行上面的 `... run`;
- 车未挪动(仍在桌前)的:同一条命令把末尾 `run` 换成 `run-docked`,跳过车段直接开始。

## 4. 状态监控界面(可选)

自带 Web 控制台,实时查看:三路相机画面、三台主机与全部服务在线状态、双臂控制器状态、
机器人 3D 数字孪生与各关节实时读数。

```bash
docker run --rm --network host -v $HOME/.ssh:/root/.ssh:ro \
  ebim-task2-offline:submission viewer
# 同网段任意电脑浏览器打开 http://<运行主机IP>:8090
```

| 概览 —— 主机 / 服务 / 双臂状态 | 相机 —— 三路画面实时监看 |
|:---:|:---:|
| ![overview](docs/img/ui_overview.jpg) | ![cameras](docs/img/ui_cameras.jpg) |

实时控制页 —— 3D 数字孪生 + 关节实时读数([现场视频](docs/img/ui_control.mp4)):

![control](docs/img/ui_control.gif)

## 5. 现场运行视频

现场真机部署时录制(均已消音;GIF 为加速预览,原速视频点各图下方链接)。

**完整流程** —— 车辆趋近、泊桌、贴装,一镜到底(10 倍速,[原速](docs/img/full_run.mp4)):

![full run](docs/img/full_run.gif)

**贴装近景** —— 导热垫分别贴到四个板位(2 倍速):

| ![t1](docs/img/target_1.gif) | ![t2](docs/img/target_2.gif) |
|:---:|:---:|
| 1 号位 · [原速](docs/img/target_1.mp4) | 2 号位 · [原速](docs/img/target_2.mp4) |
| ![t3](docs/img/target_3.gif) | ![t4](docs/img/target_4.gif) |
| 3 号位 · [原速](docs/img/target_3.mp4) | 4 号位 · [原速](docs/img/target_4.mp4) |

## 6. 故障恢复速查

表中 `tmrctl ...` 统一指:

```bash
docker run --rm -it --network host -v $HOME/.ssh:/root/.ssh:ro \
  ebim-task2-offline:submission tmrctl <参数>
```

| 症状 | 处置 |
|---|---|
| 臂红灯/抱死 | `tmrctl --enable left_arm`(或 right_arm) |
| 急停按过后臂连不上 | 松急停 → `tmrctl --down ctl --only arms` → `tmrctl --up ctl --only arms` → 再 enable 两臂 |
| "Connection to FCI refused" | Desk 页面 → Take over control → Activate FCI |
| 轮子不动 | `tmrctl --enable base` |
| 夹爪不动 | `tmrctl --enable grippers` |
| 客户端连不上服务器 | 参赛方确认服务器在线,重跑评测命令即可(启动时自动重连) |
| 查看机器人整体状态 | `tmrctl --status`(以话题频率栏为准) |

## 7. 附:不用 Docker 运行

前置:免密 SSH 同 §1;装 pixi(`curl -fsSL https://pixi.sh/install.sh | bash`)后仓库根
`pixi install`;Python3 依赖 `numpy pyyaml`,建议 `pip install ruckig`(缺失自动退化)。
ROS 环境自动适配(优先主机自带 `~/tmr_env.sh`,否则 /opt/ros 下的发行版)。服务器地址改
`deploy/inference/config.yaml` 的 `server:` 段。

| 功能 | Docker 子命令 | 裸机等价命令 |
|---|---|---|
| 硬件启动 | `bringup` | `bash deploy/prepare/up_and_enable_ctl.sh` |
| 完整评测(含车辆趋近) | `run` | `bash deploy/pipeline_full.sh` |
| 评测(车已在桌前) | `run-docked` | `bash deploy/pipeline.sh` |
| 监控界面 | `viewer` | `bash viewer-ui/run.sh --bind 0.0.0.0 --port 8090` |
| 机器人控制/恢复 | `tmrctl <参数>` | `pixi run tmrctl <参数>` |

## 8. 目录说明

```
Dockerfile / docker/  客户端容器:入口 / 自检 / 工具垫片
Dockerfile.server / server/  推理服务器容器(仅 §3.2 本地 GPU 方式用)
deploy/               评测入口(pipeline.sh / pipeline_full.sh)与推理客户端
prepare/sim2real_cal/ 车辆自动趋近就位
controller/           tmrctl:机器人服务启动/使能/离散控制/状态监控
smoother/             关节限速平滑(内部使用)
teleop/               右臂锁定用中继(内部使用)
viewer-ui/            三路相机监控前端(可选)
docs/                 官方双主机运行时指南
pixi.toml             控制工具环境(裸机方式用)
```
