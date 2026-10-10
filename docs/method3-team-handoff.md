# 方式三团队接手说明（2026-10-10）

## 先读当前状态

当前任务是 **方式三：后车使用本车 Kinect/RGB-D，识别并跟随前面的 Turtlebot**。不是方式四的前车 AMCL 位姿通信跟随。视觉跟随不需要前车 IP、共享 ROS master 或共同地图；统一任务开始/结束、全队急停另需任务接口。仓库中的 follower 要持续收到 mission 心跳，不能把“不需要车间网络”理解成可以省略任务状态。

代码目前包含此前 TonyPi 跟随的检测、关联、导航、门控及标定工具。**Turtlebot 目标适配尚未完成**：现场画面中黄色框仍可能框住 TonyPi。现有模板不能当作 Turtlebot 模板直接使用，也不能直接开启自动跟随。

已确认：Ubuntu 16.04 / ROS Kinetic / Python 2.7，Kinect RGB 和配准深度可用；真实 catkin 构建及隔离 ROS 导航冒烟通过。底盘独立限时测试分别移动约 2.1 cm、12.0 cm，并自动停车。这是底盘移动证据，不是视觉跟随成功。完整证据见 [验证记录](validation.md)。

尚未解决：相机近场两侧盲区使 DWA 足迹进入未知格，当前图上厘米级直行也可能被拒绝。不能通过反复清图、关闭未知检查或粗化轨迹采样掩盖问题。实测刹停参数、转弯/遮挡/干扰/终点和完整赛道验收仍未完成，三个 `*_calibrated` 标志保持 false。

## 代码从哪里取

使用 `aliensco/intelligent_robot_system` 的 `codex/method3-rgbd-following` 分支，不要默认克隆主分支后以为拿到了本轮修改。在 Ubuntu 终端中新建工作目录：

```bash
git clone --branch codex/method3-rgbd-following --single-branch \
  https://github.com/aliensco/intelligent_robot_system.git
cd intelligent_robot_system
git rev-parse HEAD
git switch -c team/turtlebot-visual-follow
```

旧版 Git 没有 `switch` 时使用 `git checkout -b team/turtlebot-visual-follow`。已有本地改动先保存，禁止用 `reset --hard` 覆盖队友工作。没有仓库写权限时，队友可 Fork 后在自己的仓库推送分支、提交 PR；不需要交换 GitHub 密码或 SSH 私钥。

## 队友电脑要配置什么

继续使用已验证的 Ubuntu 16.04 / ROS Kinetic 虚拟机最容易复现。Windows 上的 Python 环境仅用于离线检查，不能替代 ROS 的 Python 2.7/cv_bridge。旧系统的软件源问题应修复签名/源配置，不要关闭软件包签名验证。

在一台已安装 ROS Kinetic 的 Ubuntu 中安装依赖：

```bash
source /opt/ros/kinetic/setup.bash
sudo apt-get install git python-catkin-pkg python-yaml python-numpy python-opencv \
  ros-kinetic-cv-bridge ros-kinetic-message-filters ros-kinetic-image-view \
  ros-kinetic-tf2-ros ros-kinetic-actionlib ros-kinetic-move-base-msgs \
  ros-kinetic-move-base ros-kinetic-costmap-2d ros-kinetic-navfn \
  ros-kinetic-dwa-local-planner ros-kinetic-rosbag ros-kinetic-rostest \
  ros-kinetic-freenect-launch ros-kinetic-kobuki-node
mkdir -p ~/method3_ws/src
ln -s ~/intelligent_robot_system/turtlebot/follow3/rgbd_following \
  ~/method3_ws/src/rgbd_following
cd ~/method3_ws
catkin_make
source devel/setup.bash
rospack find rgbd_following
```

以上假设仓库克隆在家目录，且新工作空间无同名链接。不要复制旧电脑的 `build/`、`devel/` 或 Windows `.venv`，应重新构建。每个新终端都要 source 工作空间。若复制 Windows 文件遇到 `python\r` 错误，检查 CRLF 和脚本执行权限。

连接真机时，VMware 的 Kinect 和 Kobuki USB 设备均需连接到 Ubuntu 虚拟机。先确认 `lsusb`、底盘串口设备、ROS 节点和实际话题，不能只看到话题名就认定设备有数据。本次话题是：

| 数据 | 本次现场话题 |
|---|---|
| RGB | `/camera/rgb/image_rect_color` |
| 配准深度 | `/camera/depth_registered/hw_registered/image_rect` |
| RGB 内参 | `/camera/rgb/camera_info` |
| 本车里程计 | `/odom` |
| 底盘状态 | `/mobile_base/sensors/core` |

本次 `/codex_diagnostics/disabled_velocity_input` **实际连接底盘**，名字中的 disabled 不代表安全。换电脑必须重新查 `rostopic info` 确认驱动输入，不能照抄该名字。

## 除代码外还要交给队友的资料

本次另外打包了 `method3-team-handoff-20261010.tar.gz`，由当前操作者私下转交，未提交 Git：

- `micro.identity.local.yaml`：含 13 个 TonyPi 模板的现场参考配置，三个标定标志仍为 false；微动间距/限速是调试值，不是比赛参数。
- `motion.local.yaml`：本轮初始现场参考配置。
- `current_mount.json`、`floor_check.json`：本车安装变换及地面测量证据；不同车必须复测，不能只因型号相同就认定外参一致。
- `identity_check_current/`：12 组 TonyPi RGB/深度、人工 ROI、评分及溯源报告。它们用于复现旧识别或作为 Turtlebot 新任务的干扰样本，不是前车 Turtlebot 的正样本。
- 两次实际移动的 JSON 记录。

解压到独立数据目录，保留原件。该包不包含完整历史数据集、完整录包、ROS 安装、SSH 密钥或账号凭据；报告引用的其他历史负样本需另向操作者索取。配置中的旧绝对路径须按新电脑修改。原虚拟机运行目录为 `/home/haiden/codex_method3_runtime/motion_20261010`，源码运行副本在 `/home/haiden/codex_method3_motion_20261010`；它们不是队友机器的固定路径。

## 接手后的工作顺序

1. **先无运动复现。** 运行 `roslaunch rgbd_following demo.launch` 验证基础链路；退出演练后再连接真机。实车先用 `identity_preview.launch` 预览，不开启 execute。
2. **适配前车 Turtlebot。** 新建独立目标配置，检查 `perception.proposal_mode: depth_body` 的地面/支撑假设是否适合底盘、层板、支柱的稀疏结构；实测宽高后调整 `perception.body` 范围，必要时修改候选检测，不能只加模板就声称完成。
3. **重新采集和标注。** 前车背面、侧面、斜侧、不同距离；另外采集 TonyPi、椅子、另一台相似底盘、无目标、遮挡等负样本。使用 `tools/enroll_rgbd_identity.py --help` 登记，留出未参与登记的独立样本评估；该工具会生成新的 references 列表，注意不要无意覆盖其他目标配置。
4. **复核本车几何和覆盖。** 内参、深度尺度、相机到 `base_footprint` 的 TF、车体外廓、近场未知格、USB 连续运行和实际制动。相似外观的多台 Turtlebot 还需要目标关联及歧义停车验证。
5. **再做有界低速实测。** 先确认直行、目标丢失停车、前车停止后间距，再扩展转弯和绕障。`tools/run_rgbd_bounded_trial.py` 接收真实影子门控，不会替代前车检测和地图条件。本次直接底盘脉冲脚本属于临时现场诊断，不是自动跟随入口。

影子模式示例（必须先启动实际相机/底盘，具备正确安装 TF，并替换配置路径）：

```bash
roslaunch rgbd_following follow.launch robot_ns:=tb3 execute:=false \
  settings:=/absolute/path/turtlebot-target.local.yaml \
  depth_topic:=/camera/depth_registered/hw_registered/image_rect
```

没有有效的目标、任务心跳或地图覆盖时输出零速度是预期行为。统一启动/结束可用本机任务源或全队网络任务源，仍需符合 `/competition/mission`、`/competition/estop` 的接口约定，详见 [接口说明](interfaces.md)。

## 如何协作

把“代码分支链接 + 本说明 + 私下转交的数据压缩包”一起发给队友。建议各自使用独立 Git 分支：目标检测/标定、地图与避障、现场集成分别提交 PR。每次交接记录 commit、底盘型号/序列号、配置文件、数据包、已通过和未通过项目，避免只发一份改过的文件夹而不知道运行的是哪个版本。
