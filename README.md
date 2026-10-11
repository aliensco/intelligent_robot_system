# 智能机器人多机系统协同比赛

TonyPi 识别并抱起物体，沿标定路线抱物行走、左转和右转，到终点放下；Turtlebot2 采用不同方式完成跟随。四个小组分工中，**我们负责方式三：RGB-D 实时检测与跟随 TonyPi，并完成避障**。仓库已有 TonyPi 和方式二代码可作参考，不代表本组当前分工。

**当前场地调试（2026-10-11）：方式三使用本车 RGB-D 跟随前方 Turtlebot。**
运行命令与可视化见 [方式三场地入口](turtlebot/follow3/README.md)，
最新遮挡识别修改、性能数据及尚未通过的验收项见 [现场诊断](docs/method3_vision_field_diagnosis_20261011.md)。

**方式三队员直接从 GitHub 部署：** 使用 [codex/method3-team-handoff 分支](https://github.com/aliensco/intelligent_robot_system/tree/codex/method3-team-handoff)，已包含当前 Turtlebot 模板和相机外参，不需要另取压缩包。

```bash
git clone --branch codex/method3-team-handoff --single-branch \
  https://github.com/aliensco/intelligent_robot_system.git ~/method3-team-20261011
cd ~/method3-team-20261011
bash install.sh
bash run.sh --view-only
# 开始运动跟随与避障：bash run.sh；Ctrl+C 结束
```

环境为 Ubuntu 16.04 + ROS Kinetic。[快速启动](docs/method3-package-quickstart.md) 与 [团队交接](docs/method3-team-handoff.md) 说明运行、调参和继续开发；`profiles/team_current/` 保存共享配置，`runtime/` 保存安装后本机的可编辑副本。

方式三已编写颜色与深度候选识别、连续目标关联、轨迹跟随、滚动里程计地图导航和独立速度门控，尚未实机验收。默认颜色只是示例，不能当作已可靠识别 TonyPi。只有虚拟机时，从 [方式三教程与联调说明](turtlebot/follow3/README.md) 的 `demo.launch` 开始；完整入口为 `follow.launch`，默认仅输出预览速度，实机执行需先标定。

方式三进一步加入多视角外观模板与有界重识别、相机视野约束、实测速度制动包络和 VoxelLayer；配套登记/独立样本评估、录包回放、赛道验收报告与 Kinetic 集成测试入口。实机执行要求登记真实身份模板并验证标定，完整比赛验收仍需设备和四组联调。

**当前交付是可调试代码与配置，尚未实机验收。** 已有三个原始动作组，另有右转、放下的候选动作。默认不会驱动舵机；必须先完成动作、颜色、路线和多机 TF 标定。Windows 上的单元测试不能替代 ROS/Kinect/底盘测试。

## 先看哪些文件

| 内容 | 位置 |
|---|---|
| 比赛要求原文 | [第五六章实验要求](第五-六章-基于视觉与AI定位的多机协同控制实验.docx) |
| 人形机器人比赛入口 | [competition.py](humanoid/scripts/competition.py) |
| 颜色、动作、步数、心跳配置 | [competition.yaml](humanoid/config/competition.yaml) |
| 原始动作与候选动作说明 | [动作标定指南](humanoid/ActionGroup/README.md) |
| 建图避障 ROS 包 | [multi_robot_following](turtlebot/follow2/multi_robot_following) |
| 多机网络、话题与 TF | [接口约定](docs/interfaces.md) |
| 按顺序进行现场调试 | [现场操作手册](docs/field_runbook.md) |
| 报告方案与实测记录位置 | [方案设计](docs/design.md)、[验收记录](docs/validation.md) |
| 课程原始资料和代码 | [reference](reference/README.md) |

原有 `humanoid/scripts/grab_walking.py` 是踢球程序，含开环起步和射门逻辑，保留用于参考，**比赛请使用 `competition.py`**。

## 环境

虚拟机目前确定为 Ubuntu 16.04，ROS 按 Kinetic/Python 2.7 编写。先实际检查：

```bash
lsb_release -a
echo "$ROS_DISTRO"
python --version
python3 --version
```

Turtlebot 节点使用 `#!/usr/bin/env python`，兼容 ROS Kinetic 的 Python 2.7；TonyPi 比赛程序使用 **Python 3.5+**，依赖 Pi 原装 Hiwonder SDK、OpenCV、NumPy 和 PyYAML，不要求 Pi 安装 ROS。不要把 ROS Kinetic 的节点切成 Python 3 直接运行。

## 代码同步

仓库：[yky666/intelligent_robot_system](https://github.com/yky666/intelligent_robot_system)。Windows 主目录就是当前比赛目录；Linux/树莓派建议用英文目录，避免 ROS 路径问题：

```bash
git clone https://github.com/yky666/intelligent_robot_system.git ~/intelligent_robot_system
cd ~/intelligent_robot_system
git pull --ff-only
```

每次修改前拉取；每位队员使用独立分支，尤其不要同时修改同一个 `.d6a` 二进制文件：

```bash
git switch -c team/humanoid-calibration
# 修改、测试并记录实测结果
git add humanoid docs
git commit -m "Calibrate carrying actions and record field measurements"
git push -u origin team/humanoid-calibration
```

旧 Git 没有 `git switch` 时用 `git checkout -b team/humanoid-calibration`。通过 GitHub Pull Request 合并，其他设备再 `git pull --ff-only`。本机 IP 等可以放入忽略的 `*.local.yaml`，共享的最终标定参数应放入有版本的配置。地图 `.yaml` 和 `.pgm` 要一起提交到 `maps/`；视频、bag、缓存和虚拟机镜像不进 Git。

如果这台 Windows 的 Git 配置了 `127.0.0.1:7897` 代理，换网络后不再需要代理时执行 `git config --local --unset http.proxy`。不要把该地址设置到其他设备上。

## TonyPi 启动

无需硬件即可演练识别状态机和整条动作序列：

```bash
cd ~/intelligent_robot_system
python3 humanoid/scripts/competition.py --simulate
```

Pi 上先确认 `python3 -c 'import cv2, numpy, yaml'` 成功。原装 SDK 的摄像头若不可用，配置 `camera: '0'` 或实际视频流 URL。`head.pan/tilt` 填 Pi 的舵机零位配置，随后固定头部进行距离标定。

1. 在 LAB_Tool 标定目标颜色，把 OpenCV 的 0～255 LAB 下限/上限填入配置。
2. 分别验证 `grab`、抱物前进、抱物左转、抱物右转、放下；按[动作指南](humanoid/ActionGroup/README.md)处理候选动作。
3. 测量一次抱物前进的距离、左右转角和脚掌摆动范围，调整 `route` 中的 `cycles`。这些次数不是米或角度。
4. 将验证后的动作复制到 `/home/pi/TonyPi/ActionGroups/`，配置动作名；核对抓取位置和所有姿态衔接。
5. 地图跟随联调时启用 `heartbeat.enabled`，将 `host` 填运行 ROS 心跳桥的电脑 IP。确认桥的 `sender_ip` 是 Pi IP、`session` 一致。
6. 完成实机验证后设置 `hardware_calibrated: true`。

仅查看视觉和打印动作，不发舵机命令：

```bash
python3 humanoid/scripts/competition.py --debug
```

无硬件模式不会向 ROS 发比赛启动心跳。实机执行：

```bash
python3 humanoid/scripts/competition.py --execute --debug
```

识别失效时等待，不凭旧观测抓取；运动后丢弃旧相机帧，再重新检测。接近超时、相机失效、动作缺失会退出并发送故障状态。动作逐帧执行，Ctrl+C 后不发下一帧；**已发给舵机的当前帧仍可能完成，不等于硬件急停**。抱物时默认锁定抓取结束帧的六个手臂舵机，避免步态改变夹持姿态。

抓取完成目前以动作执行结束为依据，未安装夹持传感器时不能自动确认成功或检测掉落。可通过 `hold_sensor_file` 接入队友的实际传感器判定：文件需要持续刷新为 `held`，`dropped`、缺失或超时会停止后续抱物动作。没有传感器时必须实测抱持稳定性；本版本没有自动返回掉落点重新抓取的功能。

## Turtlebot2 建图避障跟随

### 安装代码

Ubuntu 16.04/ROS Kinetic 虚拟机中：

```bash
source /opt/ros/kinetic/setup.bash
sudo apt-get install ros-kinetic-navigation ros-kinetic-ar-track-alvar \
  ros-kinetic-tf2-geometry-msgs python-catkin-pkg
mkdir -p ~/catkin_ws/src
ln -s ~/intelligent_robot_system/turtlebot/follow2/multi_robot_following \
  ~/catkin_ws/src/multi_robot_following
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

已有同名链接/包时先检查，避免重复建包。课程虚拟机可能已经装好依赖。Turtlebot 的 bringup、OpenNI、深度转激光或雷达驱动沿用课程环境，未打包进本仓库。

### 建图

先按课程单车方式启动底盘、RGB-D/雷达，再运行：

```bash
roslaunch turtlebot_navigation gmapping_demo.launch
# 只在赛前建图时使用键盘遥控
roslaunch turtlebot_teleop keyboard_teleop.launch
rosrun map_server map_saver -f ~/intelligent_robot_system/maps/competition
```

场地布置改变后重建地图。结束建图并停止 gmapping、遥控和旧导航节点；比赛使用固定地图 + AMCL，不能同时让 gmapping 和 AMCL 发布同一 map→odom。

### 单台先调通，再扩展命名空间

如果目前课程驱动仍是全局话题 `/scan`、`/odom` 和 TF `odom`、`base_footprint`，使用以下参数启动 **一台**；地图路径必须是已保存的实际文件：

```bash
roslaunch multi_robot_following navigation.launch \
  robot_ns:='' map_file:=$HOME/intelligent_robot_system/maps/competition.yaml \
  base_frame:=base_footprint odom_frame:=odom scan_topic:=/scan
roslaunch multi_robot_following ar_tracker.launch robot_ns:='' \
  image_topic:=/camera/rgb/image_raw info_topic:=/camera/rgb/camera_info \
  optical_frame:=camera_rgb_optical_frame marker_size_cm:=9.0
roslaunch multi_robot_following follow_map.launch robot_ns:='' \
  base_frame:=base_footprint marker_topic:=/ar_pose_marker scan_topic:=/scan \
  cmd_vel_topic:=/cmd_vel_mux/input/navi marker_id:=1
```

相机若只有 `/camera/rgb/image_color`，调整 `image_topic`。标记必须是 **ALVAR AR 标签**，标签边长单位为 cm；普通 QR 二维码或 AprilTag 不直接兼容这个检测器。ID 必须与打印标签一致。

启动后用 RViz 的 **2D Pose Estimate** 设置 AMCL 初始位置。单车模式把 RViz 初始位姿话题设为 `/initialpose`；多机分别改为 `/tbN/initialpose`。本程序不凭动作次数估算 TonyPi 地图位置，而是把摄像头真实观测通过带时间戳的 TF 变换到地图。

### 多机与串行跟随

先按照[接口约定](docs/interfaces.md)让每车的驱动发布独立话题和独立 TF，所有车定位到**同一张同一原点的地图**。只在 launch 外包一层命名空间不能自动修正驱动的硬编码话题和 TF。

例：tb2 直接观测 TonyPi 背后的标记，方式二导航绕障：

```bash
roslaunch multi_robot_following navigation.launch robot_ns:=tb2 \
  map_file:=$HOME/intelligent_robot_system/maps/competition.yaml
roslaunch multi_robot_following ar_tracker.launch robot_ns:=tb2 marker_size_cm:=9.0
roslaunch multi_robot_following follow_map.launch robot_ns:=tb2 target_source:=marker marker_id:=1
```

如果 tb1 在前方遮挡 TonyPi，tb2 可跟随 tb1 的地图位姿。tb1 必须先有有效 AMCL 和定期扫描更新；同理可用于后续串行跟随：

```bash
roslaunch multi_robot_following follow_map.launch robot_ns:=tb2 \
  target_source:=pose leader_pose_topic:=/tb1/amcl_pose follow_distance:=0.9
```

每个 follower 的实时目标是其直接前车；不要同时启动 `marker` 和 `pose` 两种目标源。用 `target_source:=pose` 实现的是队列跟随变体；评分时“每台不同方式”的归类需按你们最终方案说明，不能靠不同车号自动算成不同方法。

ROS 主机只启动一个心跳桥：

```bash
roslaunch multi_robot_following mission_bridge.launch sender_ip:=192.168.1.101 session:=team1
```

Pi 配置心跳目标为此主机 IP。收到 `carrying` 后后车才允许运动；队列前车尚未走出足够轨迹时，后车等待，逐车自动出发。收到故障/停止，或心跳中断超过 1 秒，门控停车。`done` 后后车到最后一个带间隔的跟随点再停止，因此终点区必须容纳**整个队列**。Pi 任务结束后保持进程运行，等所有后车停稳再退出。

### 急停与验证

```bash
# 保持发布停止；该话题停止所有接入本门控的 Turtlebot
rostopic pub -r 10 /competition/estop std_msgs/Bool 'data: true'
rostopic echo /tb2/follow_status
rostopic info /tb2/cmd_vel_mux/input/navi
rostopic hz /tb2/scan
rosrun tf tf_echo map tb2/base_footprint
```

解除停车前先停止上述发布进程，再持续发布 `data: false` 并排除故障。这个 ROS 急停话题**不控制 TonyPi**，人形机器人需在 Pi 终端停止程序或使用硬件停止手段。不要启动键盘遥控、原课程 `ar_location` 控制节点或其他向底盘直接发布速度的程序；否则会绕过门控。门控应是底盘导航输入的唯一发布者。

目标丢失会取消导航并原地等待重新识别，定位差/TF 老化/扫描异常/近距离目标均停车。导航 ABORTED/REJECTED/LOST 会锁定停车，排除问题并按裁判规则处理后重启 follower，不能用后台恢复旋转继续盲走。所有阈值是起始值，须实测。

## 四种方式入口

- [follow1](turtlebot/follow1/README.md)：提供背贴 AR 的直接跟随参考入口，障碍靠近时停车。
- [follow3](turtlebot/follow3/README.md)：本组负责的 RGB-D 实时跟随，含诊断、目标关联、滚动地图导航与门控；待 ROS 和实机联调。
- [follow4](turtlebot/follow4/README.md)：前车地图位姿/轨迹串行跟随，复用本包的目标和导航接口。

背贴移动标记的直接跟随与实验文档“场地固定标签航路点引导”有区别。四台全部复用同一地图算法不能证明四种方式的多样性；[方案设计](docs/design.md)记录了需要队友补齐的范围。

## 离线检查

```bash
python3 -m pip install PyYAML opencv-python-headless numpy
python3 tools/validate_project.py
python3 -m unittest discover -s tests -v
python3 humanoid/scripts/competition.py --simulate
# Kinetic 的共用核心还可以单独用 Python 2 验证
python -m unittest discover -s tests -p test_core.py -v
```

树莓派有图形调试窗口时用原有 OpenCV 或 `opencv-python`，不要用 headless 包替换它。GitHub Actions 检查代码、配置、候选动作结构、Python 2 核心兼容性和控制决策；不会证明真实避障、舵机稳定性或比赛成绩。
