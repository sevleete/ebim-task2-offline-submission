# Mobile FR3 Duo / TMR 双主机使用说明

更新时间：2026-08-26  
当前目标：`172.16.0.50` 以及 `172.16.0.100` 运行机器人侧服务，`172.16.0.101` 运行 GELLO 和脚踏板 teleoperation。

## 1. 主机分工

| 主机 | 角色 | 主要设备/服务 |
|---|---|---|
| `172.16.0.50`  | TMR 底座、Zed 相机 |
| `172.16.0.100` | 机器人/服务主机 | Spine、双 FR3 controller、Robotiq 双夹爪、D405 相机 |
| `172.16.0.101` | Teleoperation 输入主机 | GELLO Duo、双脚踏板、GELLO publisher、脚踏板 bridge |

不要再使用旧的 `172.16.0.202`。当前 teleoperation 主机地址已经是 `172.16.0.101`。

两台主机都已经配置了便捷环境脚本：

```bash
source ~/tmr_env.sh
```

这个脚本会设置 ROS 2 Jazzy、overlay、`ROS_DOMAIN_ID=0`、CycloneDDS 等环境。正常情况下每个新终端执行一次即可。

## 2. 总体启动顺序

建议按下面顺序启动，每条 `ros2 launch` 命令放在一个独立终端里运行，终端保持打开。

1. 在 `.50`  启动 TMR 底座控制。
1. 在 `.50`  启动 Zed 相机控制。
2. 在 `.100` 启动 Spine 服务。
3. 在 `.100` 启动 Robotiq 双夹爪 manager。
4. 在 `.100` 启动双 FR3 机械臂 controller。
5. 在 `.101` 启动 GELLO publisher。
6. 在 `.101` 启动脚踏板控制 Spine + TMR。

## 3. 172.16.0.100 机器人侧启动命令

### 3.1 Spine 服务

```bash
ssh aup@172.16.0.100
source ~/tmr_env.sh

ros2 launch franka_spine_server spine.launch.py spine_ip:=172.16.16.10
```

检查：

```bash
source ~/tmr_env.sh
ros2 action info /franka_spine_node/move_absolute
ros2 service call /franka_spine_node/get_position franka_spine_msgs/srv/GetPosition '{}'
```

动作检查：
```bash
ssh aup@172.16.0.100：
source ~/tmr_env.sh
ros2 run franka_spine_examples spine_client_example.py
```

上电和下电：

```bash
ros2 service call /franka_spine_node/switch_on franka_spine_msgs/srv/SwitchOn
ros2 service call /franka_spine_node/switch_off franka_spine_msgs/srv/SwitchOff
```

直接发送 action：

```bash
ros2 action send_goal /franka_spine_node/move_absolute \
  franka_spine_msgs/action/MoveAbsolute \
  "{position: 0.4, velocity: 0.1, acceleration: 0.2, deceleration: 0.2}" \
  --feedback
```

### 3.3 Robotiq 双夹爪 manager

```bash
ssh aup@172.16.0.100
source ~/tmr_env.sh

ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
  config_file:=example_fr3_duo_config_robotiq.yaml
```

手动测试左夹爪：

```bash
source ~/tmr_env.sh
ros2 topic pub --once /left/gripper/gripper_client/target_gripper_width_percent \
  std_msgs/msg/Float32 "{data: 0.2}"
```

手动测试右夹爪：

```bash
source ~/tmr_env.sh
ros2 topic pub --once /right/gripper/gripper_client/target_gripper_width_percent \
  std_msgs/msg/Float32 "{data: 0.2}"
```

`0.2` 偏闭合，`0.8` 偏打开。GELLO 启动后也会向同样的左右夹爪 topic 发布目标开合比例。

### 3.4 双 FR3 机械臂 controller

```bash
ssh aup@172.16.0.100
source ~/tmr_env.sh

ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
  robot_config_file:=tmr_duo_config.yaml
```

检查：

```bash
source ~/tmr_env.sh
ros2 control list_controllers -c /left/controller_manager
ros2 control list_controllers -c /right/controller_manager
```

注意：机械臂 controller 必须在 `.100` 上启动。`.101` 当前不能直连 `172.16.16.11/12` 两只 FR3。

碰撞检测：

在启动 controller 服务之后，新开一个终端
```bash
source /home/aup/ros2_ws_ebim/source_ebim_stack.sh

for arm in left right; do
  ros2 service call /${arm}/service_server/set_full_collision_behavior \
    franka_msgs/srv/SetFullCollisionBehavior \
    '{lower_torque_thresholds_acceleration: [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0],
      upper_torque_thresholds_acceleration: [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0],
      lower_torque_thresholds_nominal: [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0],
      upper_torque_thresholds_nominal: [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0],
      lower_force_thresholds_acceleration: [35.0, 35.0, 35.0, 30.0, 30.0, 30.0],
      upper_force_thresholds_acceleration: [50.0, 50.0, 50.0, 42.0, 42.0, 42.0],
      lower_force_thresholds_nominal: [35.0, 35.0, 35.0, 30.0, 30.0, 30.0],
      upper_force_thresholds_nominal: [50.0, 50.0, 50.0, 42.0, 42.0, 42.0]}'
done
```

### 3.5 D405 双腕相机

推荐继续使用原指南里的直接 `realsense2_camera` 启动方式：

```bash
ssh aup@172.16.0.100
source ~/tmr_env.sh

ros2 launch realsense2_camera rs_multi_camera_launch.py \
  camera_namespace1:='/' camera_name1:='wrist_camera_left' serial_no1:=_409122272639 \
  enable_sync1:=false enable_depth1:=true \
  depth_module.color_profile1:=640x480x30 depth_module.depth_profile1:=640x480x30 \
  camera_namespace2:='/' camera_name2:='wrist_camera_right' serial_no2:=_409122274492 \
  enable_sync2:=false enable_depth2:=true \
  depth_module.color_profile2:=640x480x30 depth_module.depth_profile2:=640x480x30
```

或者

```bash
ssh aup@172.16.0.100
source ~/tmr_env.sh
bash /home/aup/ros2_ws_ebim/teleoperation/launch_d405_duo.sh
```

检查：

```bash
source ~/tmr_env.sh
ros2 topic list | grep wrist_camera
ros2 topic hz /wrist_camera_left/color/image_raw
ros2 topic hz /wrist_camera_right/color/image_raw
```

相机左右 serial 需要现场最终确认。旧指南的“当前设备号”文字写的是：

```text
left  D405: 409122274492
right D405: 409122272639
```

## 4. 172.16.0.101 Teleoperation 侧启动命令

### 4.1 GELLO 控制双臂和夹爪

```bash
ssh aup@172.16.0.101
source ~/tmr_env.sh

ros2 launch franka_gello_state_publisher main.launch.py \
  config_file:=franka_gello_duo.yaml
```

GELLO 会发布：

```text
/left/gello/joint_states
/right/gello/joint_states
/left/gripper/gripper_client/target_gripper_width_percent
/right/gripper/gripper_client/target_gripper_width_percent
``` 

因此 `.100` 上需要先启动：

```text
franka_fr3_arm_controllers
franka_gripper_manager
```

检查：

```bash
source ~/tmr_env.sh
ros2 topic hz /left/gello/joint_states
ros2 topic hz /right/gello/joint_states
ros2 topic echo /left/gripper/gripper_client/target_gripper_width_percent
ros2 topic echo /right/gripper/gripper_client/target_gripper_width_percent
```

### 4.2 脚踏板控制 Spine + TMR

```bash
ssh aup@172.16.0.101
source ~/tmr_env.sh

ros2 launch tmr_pedal_teleop mobile_teleop.launch.py
```

这个 launch 会启动：

```text
pedal_state_publisher -> /pedal/state
base_bridge           -> /swerve_drive_controller/cmd_vel
spine_bridge          -> /franka_spine_node/move_absolute
```

脚踏板映射：

```text
1A -> TMR x+
2A -> TMR x-
1B -> TMR y+
2B -> TMR y-
1C -> TMR 顺时针
2C -> TMR 逆时针

1A + 2C -> Spine 上升
1C + 2A -> Spine 下降
```

检查：

```bash
source ~/tmr_env.sh
ros2 topic echo /pedal/state
ros2 topic echo /swerve_drive_controller/cmd_vel
ros2 action info /franka_spine_node/move_absolute
```

踩单个 TMR 方向踏板时，`/swerve_drive_controller/cmd_vel` 应该出现非零 `linear.x`、`linear.y` 或 `angular.z`。踩 Spine 组合时，`.101` 的 `spine_bridge` 日志应出现 jog up/down。



## 5. 172.16.0.50 / 172.16.16.50 侧启动命令
连接 User PC，并加载 ROS 环境。当前实测 `172.16.0.50` 可以 SSH，这台机器同时也有 `172.16.16.50` 地址：

```bash
ssh tmr-user@172.16.0.50
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

建议在 `screen` 中启动相机，这样 SSH 断开后相机节点仍可继续运行：

```bash
screen
```

### 5.0 TMR 底座控制

```bash
ssh tmr-user@172.16.0.50
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch franka_bringup tmrv0_2.launch.py controller_name:=swerve_drive_controller
```

检查：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic info /swerve_drive_controller/cmd_vel
```


### 5.1 启动 ZED-M Head Camera：

```bash
cd ~/ros2_ws
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedm \
  namespace:=head_camera \
  publish_tf:=false \
  serial_number:=17064700
```

其中 `serial_number:=17064700` 表示指定启动序列号为 `17064700` 的 ZED-M 相机。可以用下面命令查看当前连接的 ZED 相机序列号：

```bash
ZED_Explorer -a
```

第一次启动 `NEURAL_LIGHT` 深度模式时，ZED SDK 可能会下载并优化 AI 深度模型。终端可能显示类似：

```text
Optimizing model: neural_depth_light_5.2 Progress: ...
```

这是正常现象，等待进度完成即可。完成后后续启动通常不需要再次等待这么久。

### 5.2 监控相机是否正常

另开一个终端，加载 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

检查节点：

```bash
ros2 node list | grep head_camera
```

正常应能看到类似：

```text
/head_camera/zed
/head_camera/zed_container
/head_camera/zed_state_publisher
```

检查相机 topic：

```bash
ros2 topic list | grep head_camera
```

常用 topic：

```text
/head_camera/zed/rgb/color/rect/image
/head_camera/zed/depth/depth_registered
/head_camera/zed/point_cloud/cloud_registered
/head_camera/zed/imu/data
/head_camera/zed/status/health
```

检查 RGB、Depth 和 IMU 发布频率：

```bash
ros2 topic hz /head_camera/zed/rgb/color/rect/image
ros2 topic hz /head_camera/zed/depth/depth_registered
ros2 topic hz /head_camera/zed/imu/data
```

正常情况下，RGB 和 Depth 大约为 14-15 Hz，IMU 大约为 100 Hz。

也可以查看诊断状态：

```bash
ros2 topic echo /diagnostics --once
```

如果看到 `Camera grabbing`，表示相机正在正常采集。

### 5.3 查看图像采集窗口

如果在目标机桌面上操作，打开一个新终端并运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /head_camera/zed/rgb/color/rect/image
```

查看深度图：

```bash
ros2 run rqt_image_view rqt_image_view /head_camera/zed/depth/depth_registered
```

如果是通过 SSH 登录，但希望窗口显示在目标机自己的显示器上，先设置目标机桌面显示环境：

```bash
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /head_camera/zed/rgb/color/rect/image
```

也可以用 RViz 查看：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

RViz 打开后：

- `Fixed Frame` 可以先设置为 `zed_left_camera_frame_optical`。
- 添加 `Image`，topic 选择 `/head_camera/zed/rgb/color/rect/image`。
- 添加 `PointCloud2`，topic 选择 `/head_camera/zed/point_cloud/cloud_registered`。

如果通过 SSH 启动 RViz 并显示到目标机屏幕上，同样先设置：

```bash
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
rviz2
```

### 5.4 修改相机参数

可以在相机运行时临时修改带 `[DYNAMIC]` 的参数。修改前先确认节点存在：

```bash
ros2 node list | grep /head_camera/zed
```

关闭自动曝光并手动设置曝光、增益：

```bash
ros2 param set /head_camera/zed video.auto_exposure_gain false
ros2 param set /head_camera/zed video.exposure 60
ros2 param set /head_camera/zed video.gain 40
```

关闭自动白平衡并手动设置白平衡：

```bash
ros2 param set /head_camera/zed video.auto_whitebalance false
ros2 param set /head_camera/zed video.whitebalance_temperature 45
```

修改图像发布频率、点云频率、深度置信度：

```bash
ros2 param set /head_camera/zed general.pub_frame_rate 10.0
ros2 param set /head_camera/zed depth.point_cloud_freq 5.0
ros2 param set /head_camera/zed depth.depth_confidence 80
```

运行时修改重启后不会保留。如果需要每次启动都生效，创建一个 override 参数文件：

```bash
nano ~/ros2_ws/zedm_override.yaml
```

示例内容：

```yaml
/**:
  ros__parameters:
    general:
      pub_frame_rate: 10.0
      pub_downscale_factor: 2.0
    video:
      auto_exposure_gain: false
      exposure: 60
      gain: 40
      auto_whitebalance: false
      whitebalance_temperature: 45
    depth:
      point_cloud_freq: 5.0
      depth_confidence: 80
      max_depth: 6.0
```

启动时加载该参数文件：

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedm \
  namespace:=head_camera \
  publish_tf:=false \
  serial_number:=19305269 \
  ros_params_override_path:=/home/tmr-user/ros2_ws/zedm_override.yaml
```

ZED-M 默认参数文件位置：

```bash
/home/tmr-user/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml
/home/tmr-user/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/zedm.yaml
```

建议先用 `ros2 param set` 临时调试参数，确认效果后再写入 override yaml。

## 6. 关键配置文件路径

### 6.1 通用环境和 DDS

`.100`：

```text
/home/aup/tmr_env.sh
/home/aup/cyclonedds.xml
```

`.101`：

```text
/home/aup/tmr_env.sh
/home/aup/cyclonedds.xml
```

当前 DDS 绑定：

```text
.100: 172.16.0.100
.101: 172.16.0.101
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 6.2 TMR 底座

`.100`：

```text

/home/aup/ros2_ws_ebim/franka_ros2/franka_bringup/config/tmr.config.yaml
/home/aup/ros2_ws_ebim/franka_ros2/franka_bringup/config/controllers.yaml

/home/aup/recloned_sources/franka_ros2_jazzy_ws/src/franka_bringup/config/tmr.config.yaml
/home/aup/recloned_sources/franka_ros2_jazzy_ws/src/franka_bringup/config/controllers.yaml
```

关键 topic：

```text
/swerve_drive_controller/cmd_vel
```

### 6.3 Spine

`.100`：

```text
/home/aup/ros2_ws_ebim/franka_ros2/franka_spine/franka_spine_server/config/franka_spine_node.yaml

/home/aup/recloned_sources/franka_ros2_jazzy_ws/src/franka_spine/franka_spine_server/config/franka_spine_node.yaml
```

关键接口：

```text
Spine/TMR IP: 172.16.16.10
/franka_spine_node/move_absolute
/franka_spine_node/get_position
/franka_spine_node/switch_on
/franka_spine_node/get_parameters_spine
```

### 6.4 双 FR3 机械臂左右配置

`.100`：

```text
/home/aup/ros2_ws_ebim/teleoperation/src/franka_fr3_arm_controllers/config/tmr_duo_config.yaml
/home/aup/ros2_ws_ebim/teleoperation/src/franka_fr3_arm_controllers/config/controllers.yaml
/home/aup/ros2_ws_ebim/teleoperation/src/franka_fr3_arm_controllers/config/tmr_left_config.yaml
/home/aup/ros2_ws_ebim/teleoperation/src/franka_fr3_arm_controllers/config/tmr_right_config.yaml


/home/aup/recloned_sources/teleoperation_overlay/src/franka_fr3_arm_controllers/config/tmr_duo_config.yaml
/home/aup/recloned_sources/teleoperation_overlay/src/franka_fr3_arm_controllers/config/controllers.yaml
/home/aup/recloned_sources/teleoperation_overlay/src/franka_fr3_arm_controllers/config/tmr_left_config.yaml
/home/aup/recloned_sources/teleoperation_overlay/src/franka_fr3_arm_controllers/config/tmr_right_config.yaml
```

当前 `tmr_duo_config.yaml` 里的左右映射：

```text
LEFT:
  namespace: left
  arm_prefix: left
  robot_ip: 172.16.16.12

RIGHT:
  namespace: right
  arm_prefix: right
  robot_ip: 172.16.16.11
```

这个映射和上游 example 可能相反，不要随手改回 example 的左右关系。

### 6.5 GELLO 左右配置

`.101` 源码配置：

```text
/home/aup/ros2_ws_ebim/teleoperation/src/franka_gello_state_publisher/config/franka_gello_duo.yaml


/home/aup/ros2_ws/teleoperation/src/franka_gello_state_publisher/config/franka_gello_duo.yaml
```

`.101` launch 实际读取的安装配置：

```text
/home/aup/ros2_ws_ebim/install/franka_gello_state_publisher/share/franka_gello_state_publisher/config/franka_gello_duo.yaml

/home/aup/ros2_ws/teleoperation/install/franka_gello_state_publisher/share/franka_gello_state_publisher/config/franka_gello_duo.yaml
```

当前安装配置是 symlink，指向：

```text
/home/aup/ros2_ws_ebim/build/franka_gello_state_publisher/config/franka_gello_duo.yaml

/home/aup/ros2_ws/teleoperation/build/franka_gello_state_publisher/config/franka_gello_duo.yaml
```

当前 GELLO 左右映射：

```text
LEFT:
  namespace: left
  frame_id: left_fr3v2_link0
  joint_names: left_fr3v2_joint1 ... left_fr3v2_joint7
  com_port: usb-ROBOTIS_OpenRB-150_A957F74D503059384C2E3120FF062E1D-if00

RIGHT:
  namespace: right
  frame_id: right_fr3v2_link0
  joint_names: right_fr3v2_joint1 ... right_fr3v2_joint7
  com_port: usb-ROBOTIS_OpenRB-150_46BEEC57503059384C2E3120FF072707-if00
```

`com_port` 是相对 `/dev/serial/by-id/` 的名字。当前设备查询结果：

```text
/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_A957F74D503059384C2E3120FF062E1D-if00 -> ttyACM0
/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_46BEEC57503059384C2E3120FF072707-if00 -> ttyACM1
```

查询命令：

```bash
ls -l /dev/serial/by-id/ | grep -i ROBOTIS
```

### 6.6 Robotiq 双夹爪左右配置

`.100`：

```text
/home/aup/ros2_ws_ebim/teleoperation/src/franka_gripper_manager/config/example_fr3_duo_config_robotiq.yaml
/home/aup/ros2_ws_ebim/teleoperation/src/franka_gripper_manager/config/robotiq_controllers.yaml

/home/aup/recloned_sources/teleoperation_overlay/src/franka_gripper_manager/config/example_fr3_duo_config_robotiq.yaml
/home/aup/recloned_sources/teleoperation_overlay/src/franka_gripper_manager/config/robotiq_controllers.yaml
```

当前夹爪左右映射：

```text
LEFT:
  namespace: left/gripper
  com_port: /dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAANTK6Q-if00-port0

RIGHT:
  namespace: right/gripper
  com_port: /dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAANVRU5-if00-port0
```

查询命令：

```bash
ls -l /dev/serial/by-id/ | grep -i FTDI
```

### 6.7 脚踏板配置

`.101` 脚踏板 teleop 配置：

```text
/home/aup/ros2_ws/teleoperation/src/tmr_pedal_teleop/config/pedal_map.yaml
```

`.101` launch 实际读取的安装配置：

```text
/home/aup/ros2_ws/teleoperation/install/tmr_pedal_teleop/share/tmr_pedal_teleop/config/pedal_map.yaml
```

当前安装配置是 symlink，指向：

```text
/home/aup/ros2_ws/teleoperation/build/tmr_pedal_teleop/config/pedal_map.yaml
```

脚踏板 udev 规则：

```text
/etc/udev/rules.d/90-tmr-footswitch-202.rules
```

注意：文件名里仍然有 `202`，但这是历史文件名；规则当前在 `.101` 上生效，并创建下面两个 symlink：

```text
/dev/tmr_foot_pedal_1 -> input/event12
/dev/tmr_foot_pedal_2 -> input/event9
```

当前 udev 绑定依据：

```text
tmr_foot_pedal_1: ID_PATH=pci-0000:bd:00.4-usb-0:1.3:1.0
tmr_foot_pedal_2: ID_PATH=pci-0000:bd:00.4-usb-0:1.1:1.0
USB vendor/product: 3553:b001 PCsensor FootSwitch
```

查询命令：

```bash
ls -l /dev/tmr_foot_pedal_*
udevadm info -q property -n /dev/tmr_foot_pedal_1 | grep -E 'ID_PATH|ID_VENDOR|ID_MODEL|DEVLINKS'
udevadm info -q property -n /dev/tmr_foot_pedal_2 | grep -E 'ID_PATH|ID_VENDOR|ID_MODEL|DEVLINKS'
```

### 6.8 D405 相机配置和左右信息

`.100` 可参考的 sensor suite 配置：

```text
/home/aup/recloned_sources/franka_ros2_jazzy_ws/src/franka_mobile_sensors/config/d405_duo_sensor_suite.yaml
/home/aup/recloned_sources/franka_ros2_jazzy_ws/src/franka_mobile_sensors/launch/cameras/realsense_cameras.launch.py
```

当前 `d405_duo_sensor_suite.yaml` 内容要点：

```text
d405_1:
  namespace: d405_1
  serial_number: _409122274492

d405_2:
  namespace: d405_2
  serial_number: _409122272639
```

原指南推荐的直接启动方式使用 topic 名：

```text
/wrist_camera_left/...
/wrist_camera_right/...
```

如果现场要求明确 left/right，请用图像方向确认后固定 serial。当前已知两个 D405 serial 是：

```text
409122274492
409122272639
```

## 7. 常用状态检查

### 7.1 看进程

`.100`：

```bash
pgrep -af "franka_spine|spine.launch.py"
pgrep -af "tmrv0_2|swerve|franka_bringup"
pgrep -af "franka_fr3_arm_controllers|joint_impedance|ros2_control_node"
pgrep -af "robotiq|gripper"
pgrep -af "realsense|rs_multi_camera"
```

`.101`：

```bash
pgrep -af "gello|franka_gello"
pgrep -af "pedal|tmr_pedal_teleop|base_bridge|spine_bridge"
```

### 7.2 看 ROS graph

任意一台主机：

```bash
source ~/tmr_env.sh
ros2 node list
ros2 topic list
ros2 action list
```

关键 topic/action：

```text
/left/gello/joint_states
/right/gello/joint_states
/left/gripper/gripper_client/target_gripper_width_percent
/right/gripper/gripper_client/target_gripper_width_percent
/pedal/state
/swerve_drive_controller/cmd_vel
/franka_spine_node/move_absolute
```

### 7.3 单独验证脚踏板到 TMR

在 `.101` 终端 A：

```bash
source ~/tmr_env.sh
ros2 launch tmr_pedal_teleop mobile_teleop.launch.py
```

在任意主机终端 B：

```bash
source ~/tmr_env.sh
ros2 topic echo /swerve_drive_controller/cmd_vel
```

踩 `1A`，应看到 `linear.x` 为正；踩 `2A`，应看到 `linear.x` 为负。

如果 `/pedal/state` 有变化，但 `/swerve_drive_controller/cmd_vel` 没有变化，检查 `.101` 的 `base_bridge`。

如果 `/swerve_drive_controller/cmd_vel` 有非零速度，但 TMR 不动，检查 `.100` 的 `swerve_drive_controller` 和 TMR 硬件状态。

### 7.4 单独验证 GELLO 到夹爪

`.100` 先启动 gripper manager，`.101` 启动 GELLO 后，在任意主机检查：

```bash
source ~/tmr_env.sh
ros2 topic echo /left/gripper/gripper_client/target_gripper_width_percent
ros2 topic echo /right/gripper/gripper_client/target_gripper_width_percent
```

操作 GELLO 夹爪，应看到数值变化。

### 7.5 后续数据采集查询接口

采集前先加载环境：

```bash
# .100 / .101
source ~/tmr_env.sh

# .50
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

大流量相机建议在设备所在主机就近录制，低频状态量可以在任意能看到 ROS graph 的主机录制。当前核对到的服务分布是：

```text
.50 : TMR base, ZED-M head camera
.100: Spine, Robotiq grippers, D405 wrist cameras
.101: GELLO, pedal bridge
```

常用发现命令：

```bash
ros2 topic list -t | grep -E "head_camera|wrist_camera|gello|gripper|franka_robot_state|pedal|swerve|olive"
ros2 service list -t | grep -E "franka_spine_node|device_info|controller_manager"
ros2 action list -t | grep -E "spine|robotiq|action_server"
ros2 topic info /topic_name
ros2 topic hz /topic_name
ros2 interface show package/msg/TypeName
```

#### 7.5.1 可直接订阅或录包的 topic

| 数据 | 接口 | 类型 | 备注 |
|---|---|---|---|
| ZED 头部 RGB | `/head_camera/zed/rgb/color/rect/image` | `sensor_msgs/msg/Image` | `.50`，当前可见 |
| ZED 头部 RGB 参数 | `/head_camera/zed/rgb/color/rect/camera_info` | `sensor_msgs/msg/CameraInfo` | `.50` |
| ZED 头部 IMU | `/head_camera/zed/imu/data` | `sensor_msgs/msg/Imu` | `.50` |
| ZED 健康状态 | `/head_camera/zed/status/health` | `zed_msgs/msg/HealthStatusStamped` | `.50` |
| ZED heartbeat | `/head_camera/zed/status/heartbeat` | `zed_msgs/msg/Heartbeat` | `.50` |
| ZED 深度/点云 | `/head_camera/zed/depth/depth_registered`, `/head_camera/zed/point_cloud/cloud_registered` | `sensor_msgs/msg/Image`, `sensor_msgs/msg/PointCloud2` | 本次核对未看到数据 topic，只看到 depth camera_info；需要深度时先确认 ZED depth/point cloud 参数 |
| 左 D405 RGB | `/wrist_camera_left/color/image_raw` | `sensor_msgs/msg/Image` | `.100` |
| 左 D405 Depth | `/wrist_camera_left/depth/image_rect_raw` | `sensor_msgs/msg/Image` | `.100` |
| 左 D405 相机参数 | `/wrist_camera_left/color/camera_info`, `/wrist_camera_left/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `.100` |
| 左 D405 metadata | `/wrist_camera_left/color/metadata`, `/wrist_camera_left/depth/metadata` | `realsense2_camera_msgs/msg/Metadata` | `.100` |
| 右 D405 RGB | `/wrist_camera_right/color/image_raw` | `sensor_msgs/msg/Image` | `.100` |
| 右 D405 Depth | `/wrist_camera_right/depth/image_rect_raw` | `sensor_msgs/msg/Image` | `.100` |
| 右 D405 相机参数 | `/wrist_camera_right/color/camera_info`, `/wrist_camera_right/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `.100` |
| 右 D405 metadata | `/wrist_camera_right/color/metadata`, `/wrist_camera_right/depth/metadata` | `realsense2_camera_msgs/msg/Metadata` | `.100` |
| GELLO 左右 7 轴 | `/left/gello/joint_states`, `/right/gello/joint_states` | `sensor_msgs/msg/JointState` | `.101` |
| GELLO/夹爪目标开合 | `/left/gripper/gripper_client/target_gripper_width_percent`, `/right/gripper/gripper_client/target_gripper_width_percent` | `std_msgs/msg/Float32` | 目标值，0-1 比例 |
| Robotiq 左右实际关节 | `/left/gripper/joint_states`, `/right/gripper/joint_states` | `sensor_msgs/msg/JointState` | `.100` |
| 脚踏板状态 | `/pedal/state` | `std_msgs/msg/String` | `.101` |
| 底座里程计 | `/swerve_drive_controller/odom` | `nav_msgs/msg/Odometry` | `.50` |
| 底座速度命令 | `/swerve_drive_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | bridge 输出给 controller |
| 底座速度输出 | `/swerve_drive_controller/cmd_vel_out` | `geometry_msgs/msg/TwistStamped` | controller 输出 |
| OlixSense IMU | `/olive/olixSense/x1/id001/imu` | `sensor_msgs/msg/Imu` | `.50`，如现场启用 |
| OlixSense 加速度 | `/olive/olixSense/x1/id001/acceleration` | `geometry_msgs/msg/AccelStamped` | `.50`，如现场启用 |
| OlixSense 速度 | `/olive/olixSense/x1/id001/velocity` | `geometry_msgs/msg/TwistStamped` | `.50`，如现场启用 |
| 全局诊断 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 相机、控制器等诊断 |
| 坐标变换 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 需要重建坐标关系时录制 |

双 FR3 controller 启动后，建议额外录下面这些机械臂状态 topic。本次核对 `.100` 时未看到 `franka_fr3_arm_controllers` 进程，所以下面这组要以 controller 实际启动后的 `ros2 topic list -t` 为准：

```text
/left/franka_robot_state_broadcaster/robot_state
/right/franka_robot_state_broadcaster/robot_state
/left/franka_robot_state_broadcaster/current_pose
/right/franka_robot_state_broadcaster/current_pose
/left/franka_robot_state_broadcaster/measured_joint_states
/right/franka_robot_state_broadcaster/measured_joint_states
/left/franka_robot_state_broadcaster/desired_joint_states
/right/franka_robot_state_broadcaster/desired_joint_states
/left/franka_robot_state_broadcaster/external_wrench_in_base_frame
/right/franka_robot_state_broadcaster/external_wrench_in_base_frame
```

单帧查询可以直接用 `ros2 topic echo ... --once`：

```bash
ros2 topic echo /left/franka_robot_state_broadcaster/robot_state --once
ros2 topic echo /right/franka_robot_state_broadcaster/robot_state --once
ros2 topic echo /left/franka_robot_state_broadcaster/current_pose --once
ros2 topic echo /right/franka_robot_state_broadcaster/current_pose --once
ros2 topic echo /left/franka_robot_state_broadcaster/measured_joint_states --once
ros2 topic echo /right/franka_robot_state_broadcaster/measured_joint_states --once
ros2 topic echo /left/franka_robot_state_broadcaster/external_wrench_in_base_frame --once
ros2 topic echo /right/franka_robot_state_broadcaster/external_wrench_in_base_frame --once
```

如果出现下面这种输出，说明双 FR3 controller 或 `franka_robot_state_broadcaster` 当前没有启动，不是 echo 命令写错：

```text
Unknown topic '/right/franka_robot_state_broadcaster/robot_state'
WARNING: topic [/right/franka_robot_state_broadcaster/robot_state] does not appear to be published yet
Could not determine the type for the passed topic
```

这时先在 `.100` 启动双 FR3 controller：

```bash
source ~/tmr_env.sh
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
  robot_config_file:=tmr_duo_config.yaml
```

#### 7.5.2 查询型 service

Spine 没有周期状态 topic，状态/位置查询走 service：

```bash
ros2 service call /franka_spine_node/get_state franka_spine_msgs/srv/GetSpineState '{}'
ros2 service call /franka_spine_node/get_position franka_spine_msgs/srv/GetPosition '{}'
ros2 service call /franka_spine_node/get_parameters_spine franka_spine_msgs/srv/GetParameters '{}'
```

RealSense 设备信息：

```bash
ros2 service call /wrist_camera_left/device_info realsense2_camera_msgs/srv/DeviceInfo '{}'
ros2 service call /wrist_camera_right/device_info realsense2_camera_msgs/srv/DeviceInfo '{}'
```

Controller 状态查询：

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
ros2 service call /left/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
ros2 service call /right/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
ros2 service call /left/gripper/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
ros2 service call /right/gripper/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
```

ZED 自带 SVO 录制接口：

```bash
ros2 service call /head_camera/zed/start_svo_rec zed_msgs/srv/StartSvoRec \
  "{bitrate: 0, compression_mode: 0, target_framerate: 0, input_transcode: false, svo_filename: '/home/tmr-user/ros2_ws/head_camera.svo2'}"
ros2 service call /head_camera/zed/stop_svo_rec std_srvs/srv/Trigger '{}'
```

#### 7.5.3 action 接口

下面接口主要用于控制，不建议当成周期数据源；如果数据采集时需要记录动作过程，可以同时录相关状态 topic，并保存 action goal/feedback 日志。

```text
/franka_spine_node/move_absolute                         franka_spine_msgs/action/MoveAbsolute
/left/gripper/robotiq_gripper_controller/gripper_cmd     control_msgs/action/GripperCommand
/right/gripper/robotiq_gripper_controller/gripper_cmd    control_msgs/action/GripperCommand
/left/action_server/ptp_motion                           franka_msgs/action/PTPMotion
/right/action_server/ptp_motion                          franka_msgs/action/PTPMotion
/left/action_server/error_recovery                       franka_msgs/action/ErrorRecovery
/right/action_server/error_recovery                      franka_msgs/action/ErrorRecovery
```

#### 7.5.4 rosbag 示例

只录低频状态和 teleoperation 输入：

```bash
ros2 bag record -o tmr_state_$(date +%Y%m%d_%H%M%S) \
  /left/gello/joint_states \
  /right/gello/joint_states \
  /left/gripper/joint_states \
  /right/gripper/joint_states \
  /left/gripper/gripper_client/target_gripper_width_percent \
  /right/gripper/gripper_client/target_gripper_width_percent \
  /pedal/state \
  /swerve_drive_controller/odom \
  /swerve_drive_controller/cmd_vel \
  /swerve_drive_controller/cmd_vel_out \
  /diagnostics \
  /tf \
  /tf_static
```

录 D405 双腕相机：

```bash
ros2 bag record -o tmr_wrist_cameras_$(date +%Y%m%d_%H%M%S) \
  /wrist_camera_left/color/image_raw \
  /wrist_camera_left/color/camera_info \
  /wrist_camera_left/depth/image_rect_raw \
  /wrist_camera_left/depth/camera_info \
  /wrist_camera_right/color/image_raw \
  /wrist_camera_right/color/camera_info \
  /wrist_camera_right/depth/image_rect_raw \
  /wrist_camera_right/depth/camera_info
```

录 ZED 头部相机：

```bash
ros2 bag record -o tmr_head_camera_$(date +%Y%m%d_%H%M%S) \
  /head_camera/zed/rgb/color/rect/image \
  /head_camera/zed/rgb/color/rect/camera_info \
  /head_camera/zed/imu/data \
  /head_camera/zed/status/health
```

## 8. 停止命令

优先在对应终端按 `Ctrl+C` 停止。需要按功能清理时再用下面命令。

`.50`：

```bash
pkill -f "tmrv0_2.launch.py|swerve_drive_controller"
pkill -f "zed_camera.launch.py|zed_wrapper|zed_container"
```

`.100`：

```bash
pkill -f "franka_spine_server|spine_action_server_node|spine.launch.py"
pkill -f "franka_fr3_arm_controllers.launch.py|joint_impedance_controller"
pkill -f "franka_gripper_manager|robotiq"
pkill -f "rs_multi_camera_launch|realsense2_camera"
```

`.101`：

```bash
pkill -f "franka_gello_state_publisher|gello_publisher"
pkill -f "tmr_pedal_teleop|pedal_state_publisher|base_bridge|spine_bridge"
```

不要随手杀所有 `ros2_control_node`，因为 `.100` 上底座、机械臂、夹爪都可能各自使用 controller manager。

## 9. 最小可用组合

只用 GELLO 控制双臂和夹爪：

```text
.100: franka_gripper_manager
.100: franka_fr3_arm_controllers
.101: franka_gello_state_publisher
```

只用脚踏板控制 Spine 和 TMR：

```text
.100: franka_spine_server
.50 : franka_bringup tmrv0_2
.101: tmr_pedal_teleop
```

全功能：

```text
.100: franka_spine_server
.50 : franka_bringup tmrv0_2
.100: franka_gripper_manager
.100: franka_fr3_arm_controllers
.100: realsense2_camera, optional
.101: franka_gello_state_publisher
.101: tmr_pedal_teleop
```
