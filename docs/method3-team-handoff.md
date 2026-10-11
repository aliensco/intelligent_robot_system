# 方式三部署包与团队交接（2026-10-11）

本包用于 **后车使用本车 Kinect 的 RGB 图像和配准深度，识别并跟随前面的 Turtlebot，同时通过点云地图规划避障**。当前交付的是可继续现场调试、修改和复现的版本，尚未完成全赛道验收。

首次使用先按 [快速启动](method3-package-quickstart.md) 操作。本说明介绍配置、环境、协作和问题复现。

**优先用 GitHub 交接：** [codex/method3-team-handoff](https://github.com/aliensco/intelligent_robot_system/tree/codex/method3-team-handoff) 已纳入下面的源码、安装脚本及 `profiles/team_current/`。直接克隆此分支即可，不要只取主分支或较早的 `codex/method3-rgbd-following` 后认为它包含这轮更新。压缩包是此前同一套运行代码和配置的固定快照，后续改动通过 Git 提交与 PR 协作。

## 包内有什么

快速启动中的克隆目录或压缩包解压目录为 `method3-team-20261011`，下文命令均在这个目录中执行。可以放在任意用户的目录，不要求用户名为 `haiden`。

| 路径 | 用途 |
| --- | --- |
| `install.sh` | 安装缺失依赖、配置设备访问、建立工作区并构建 |
| `run.sh` | 统一启动底盘、相机、识别、点云窗口及场地跟随会话 |
| `profiles/team_current/` | 本次交付的 Turtlebot 目标模板、相机安装变换和启动配置原件 |
| `runtime/` | 安装时从上述配置复制的实际运行配置；后续在这里调参 |
| `runtime/field_sessions/` | 每次启动的日志、最终生效配置和运动录包 |
| `workspace/` | 安装器在本机生成的 catkin 工作区 |
| `turtlebot/follow3/rgbd_following/` | ROS 节点、视觉/深度融合、关联、规划、门控和清障插件源码 |
| `tools/` | 场地会话、采样、标定、离线评估和验收工具 |
| `tests/`、`tests/fixtures/` | 自动化回归测试及其样本/摘要 |
| `.git/` | 上游历史及本次包装快照，可直接创建开发分支 |

当前目标 YAML 已包含 Turtlebot 外观模板和底盘几何参考，运行时不需要另外下载模板图片。**最新一轮原始采集样本、全部历史 rosbag 不在包中**；`tests/fixtures/` 中的测试样本可用于已有回归，但不能替代完整训练集或独立场地验收数据。

## 环境与安装

部署目标是已有 **Ubuntu 16.04 + ROS Kinetic** 的虚拟机，运行节点使用系统 Python 2.7 和 ROS 配套的 `cv_bridge`。本包不是 Ubuntu/ROS 虚拟机镜像，不会安装操作系统，也不承诺可直接用于 ROS Noetic、ROS 2 或 Windows 原生 Python。

```bash
cd ~/method3-team-20261011
bash install.sh
```

如果解压到别处，将第一行替换为实际目录。安装器仅安装缺失的 apt 依赖，配置通用 Kobuki 设备规则及用户组，建立根目录下的 `workspace/` 并编译。缺少依赖或需要写设备规则时会使用 `sudo`；队员输入自己的 Ubuntu 密码。

已经配好系统依赖和 USB 访问权限，只需重新构建时：

```bash
bash install.sh --skip-system
```

这个选项不安装缺失软件、不修复系统设备权限。安装器会保留已有 `runtime/` 中的配置修改；`profiles/team_current/` 留作交付原件，更新配置时先比较差异，再决定合并哪些参数。

若安装器新添加了用户组，注销 Ubuntu 后重新登录使组成员资格生效；按安装输出提示重新连接 USB。不要复制其他电脑的 `build/`、`devel/` 或旧绝对路径符号链接，也不要用 Windows 的 `.venv` 替代 ROS 环境。

主要依赖包括：

- ROS 的 freenect、Kobuki 驱动、`cv_bridge`、TF、`move_base`、Navfn、DWA、costmap、image_view、RViz 和 rosbag。
- 系统 Python 的 NumPy、OpenCV、PyYAML，以及 catkin 构建工具。
- 启动脚本使用的 `timeout`、`flock` 和 `fuser`。

如果 Ubuntu 16.04 的旧软件源报签名或下载错误，应修复 apt 源/签名后重试安装；这不是识别算法故障。不要通过安装最新 pip OpenCV 覆盖 ROS 配套版本。

## USB 与机器人连接

在 VMware 的 **VM → Removable Devices** 中，将 `Future Devices iClebo Kobuki` 和 `Microsoft Xbox NUI Camera` 连接到 Ubuntu。Kinect 需要正常供电；USB 菜单打勾只表示设备分配，是否有连续图像由启动日志和预览确认。

每台电脑只控制本机连接的后车。方式三不需要前车 IP、前车发布定位，不需要共享 ROS master 或搭建车间通信网络。当前封装使用固定 ROS 节点/话题名，适合一台 VM 内的一套底盘和相机。全队统一开始、结束等任务通信若比赛需要，应另外接入，未包含在此本机场地入口中。

默认设备为 `/dev/kobuki`。包使用通用设备规则，不绑定旧底盘序列号。不要同时把两台 Kobuki 接进同一个 VM；多底盘或非默认串口需要一致修改驱动设备路径和会话脚本中的串口占用检查。

## 启动与停止

相机、识别框和点云预览，不启动运动转发：

```bash
bash run.sh --view-only
```

成功后应出现图像窗口和 RViz。图像窗口显示目标识别预览；RViz 显示实时障碍点云、地面清障观测、局部地图、规划路径和目标位置。预览初始化完成后命令会返回，窗口可继续保留。

开始完整的当前跟随避障会话：

```bash
bash run.sh
```

入口会自动加载本包工作区，不需要另开终端 `source`，也不需要手工分别运行底盘、freenect、旧试验脚本或清图命令。避免同时启动另一套同名节点。

需要较短的测试时：

```bash
bash run.sh --duration 60 --distance 2 --max-speed 0.08 --max-turn 0.30
```

在启动终端按 **Ctrl+C** 结束本轮运动。到达时间或里程上限也会结束；默认保留相机、底盘驱动和可视化供下一轮使用。`--shutdown-stack` 可在本轮退出时关闭本轮新启动的组件，不保证关闭之前会话复用的所有进程。

```bash
bash run.sh --help
```

### 当前默认参数

| 项目 | 当前值 | 含义 |
| --- | --- | --- |
| `--duration` | 300 秒 | 单轮运动时间上限 |
| `--distance` | 5 米 | 单轮累计行程上限，不是两点净位移 |
| `--max-speed` | 0.10 m/s | 线速度上限，实际指令可能更低 |
| `--max-turn` | 0.30 rad/s | 角速度上限 |
| `follow_distance` | 0.60 米 | 当前配置的期望跟随距离 |
| `standoff_min_distance` | 0.50 米 | 正常候选停车点允许的较小间距 |
| `minimum_distance` | 0.40 米 | 过近停止阈值 |
| `footprint_radius` | 0.25 米 | 当前规划使用的圆形车体包络半径 |

这些距离由后车定位原点和检测到的目标参考位置计算，**不是两个保险杠之间的净空，也不是固定保证的制动距离**。绕障时可能选择更远的停车点或中间点，不保证每时每刻维持 0.60 米。

当前场地入口允许规划经过未观测栅格，同时保留实测障碍、距离及输入时效检查。这是运行策略，并不表示未知区域已经由相机测得为空地。目标丢失或无法生成可执行路径时仍可能停车。

## 改哪里才会生效

主要修改：

- `runtime/turtlebot_follow_current.yaml`：跟随距离、目标模板、跟踪、融合、可见性与规划参数。
- `runtime/turtlebot_follow_current.launch`：相机到机器人坐标系的安装变换、话题及导航包装配置。
- `runtime/sensors.launch`：底盘驱动与传感器探针启动。

不要误改仓库演示用的 `config/follow.yaml` 后认为场地配置已经同步。场地 supervisor 会按命令行覆盖速度上限及部分 DWA/足迹参数；每轮日志中的 `field_settings.yaml` 和 `field.launch` 是这轮最终生效的配置，可以据此核对。

launch 中的 `settings` 文件路径应使用相对该 launch 的真实文件名，例如 `turtlebot_follow_current.yaml`。场地工具会先用 Python 读取这个 YAML，不能在这里直接使用 `$(env HOME)` 等 ROS 替换表达式。

当前相机安装变换是 **`base_footprint → camera_link`**，不是直接到 RGB 光学镜头中心。换相机安装位置后需要更新对应 TF；同型号 Turtlebot 不代表安装偏移一定相同。原始交付配置保存在 `profiles/team_current/`，便于比较。

当前底盘速度输入被重映射为 `/codex_diagnostics/disabled_velocity_input`，这个名字中的 `disabled` 不表示它没有连接电机。保持 `sensors.launch` 与场地会话脚本一致，直接发速度不能代替跟随链路验证。

编辑配置后结束当前运动，再运行 `bash run.sh` 启动新一轮。修改 Python 源码后也需要重启使用它的节点；修改 C++ 插件或构建文件后执行 `bash install.sh --skip-system` 重新构建。

## 当前进展与尚未通过的项目

已经集成：Turtlebot RGB 模板与真实深度支持、底盘/上层结构辅助、目标关联、最新帧处理、点云障碍地图、可达绕行子目标、局部规划、无进展重试，以及带图像/RViz 和日志的启动入口。现场已运行识别和有界跟随运动；自动化测试及部分录制样本回放可以复现。

**尚未证明完整比赛场地稳定通过。** 相似车辆干扰、较大转弯、连续遮挡恢复、多障碍动态跟随、跨机器安装迁移及全队终点仍需要独立实测。绕行时仍可能因视野、识别或路径条件停车，不能把“已发出速度”“已摆放障碍”“单次短距离移动”当作绕障验收完成。

配置中的 `hardware_calibrated`、`identity_calibrated` 和 `depth_coverage_calibrated` 仍保持 `false`。它们不是靠改成 `true` 就能完成的验收。请按 [自主跟随避障验收说明](method3_acceptance.md) 记录真实完成的场景；未完成场景保持 `INCOMPLETE`。

## 日志与问题复现

终端打印 `SESSION_LOG ...`，对应 `runtime/field_sessions/日期时间_进程号/`。常用文件：

| 文件 | 排查用途 |
| --- | --- |
| `camera.log`、`base.log` | 本轮新启动的相机/底盘驱动输出 |
| `follow.log` | 识别、TF、地图和导航节点输出 |
| `preview.log`、`pointcloud.log` | 图像窗口、RViz 启动问题 |
| `status.jsonl` | 识别、跟随、门控状态变化 |
| `motion.log` | 速度、里程计、编码器统计和本轮结束原因 |
| `telemetry.bag` | 目标、状态、地图、路径、TF 及底盘遥测；仅运动会话默认录制 |
| `field_settings.yaml`、`field.launch` | 本轮实际配置 |

复用旧驱动或窗口时，相应启动日志可能在前一轮目录中。默认 `telemetry.bag` 不录原始 RGB/深度图；能分析运动和规划，但不能单靠它重新运行图像识别。要调查误识别，应额外采集同步 RGB-D 并人工标注目标，保留独立验证样本。

常见现象：

| 现象 | 下一步 |
| --- | --- |
| 找不到工作区/包 | 在解压根目录重新执行 `bash install.sh --skip-system`；不要引用其他电脑的 `devel/` |
| `/dev/kobuki` 不存在或权限错误 | 检查 VMware USB 分配、规则安装及重新登录后的用户组；安装器可补规则 |
| Kinect 超时或无新图像 | 看 `camera.log`，检查 Kinect 供电与 Camera 是否交给 Ubuntu；仅有话题名不足以证明图像正常 |
| 同名节点退出、串口被占用 | 结束另一个旧启动入口后只使用 `run.sh`；不要并行手动运行第二套底盘驱动 |
| 没窗口但节点存在 | 在 Ubuntu 图形桌面的终端执行入口；检查 `preview.log` / `pointcloud.log` 和 DISPLAY |
| 前车已在附近但不走 | 先看 `status.jsonl`、`motion.log`：可能已在跟随距离内、目标未确认、数据过期或路径受阻 |
| 点云停住但 RViz 还显示画面 | 查看驱动及状态日志，RViz 保留旧图不代表传感器仍在更新 |
| `travel_budget` / `duration_limit` | 本轮达到了设置的行程/时间上限；它不是整段路线验收通过的结论 |

报告问题时一起交给队友：`git rev-parse HEAD`、实际启动命令、本轮日志目录、使用的配置、底盘/相机是否更换，以及一段能看到前后车和障碍位置的视频。注明人工扶车、移动障碍或重摆目标的时间，分析时应区分自主运动和人工移动。

## 团队继续开发

解压目录本身是带上游历史和本次包装快照的 Git 工作副本，不需要重新克隆一份旧分支再手工覆盖代码：

```bash
git status
git log -1 --oneline
git remote -v
git checkout -b team/your-name
```

从 GitHub 克隆的目录同样直接创建自己的分支即可。新电脑第一次提交前需要设置自己的 `git config user.name` 和 `git config user.email`。安装后的本机 `runtime/`、构建目录、录包和视频不提交；共享参数放在 `profiles/team_current/`，依赖清单与本次部署验证记录在 `deploy/method3/`。

用自己的姓名或任务名替换 `your-name`。检查实际 remote 和权限后再推送；有写权限可推送自己的分支，没有则 Fork 到自己的账号并添加对应 remote，再提交 PR。不要交换密码、访问令牌或 SSH 私钥。

源码改动正常提交。`runtime/` 是现场工作配置，若其被忽略，不能以一次普通 `git commit` 就当作已交付；将经过复核的配置改动同步到约定的受版本管理 profile 并提交，或另附配置及差异说明。提交前检查 `git diff`，避免混入录包、工作区构建产物或凭据。

可先在安装好依赖的环境运行规划回归示例：

```bash
python -m unittest discover -s tests -p 'test_rgbd_planning.py'
```

更多离线测试见 `tests/`；部分节点替身及离线工具测试要求 Python 3，在 Python 2 下会跳过。`tools/calibrate_rgbd_target.py` 和 `tools/validate_project.py` 是 Python 3 离线工具，场地启动与当前模板运行使用 Python 2.7。离线开发环境和 ROS Kinetic 实机运行环境应分别说明，不能用“有跳过项的测试通过”代替 ROS 构建和实车验证。

建议按识别/标定、地图/规划、场地集成分别开发，PR 附复现命令、测试和对应日志。每次交接至少保留“代码版本 + 当前配置 + 已完成/未完成的场景”，下一位才能复现同一套行为。
