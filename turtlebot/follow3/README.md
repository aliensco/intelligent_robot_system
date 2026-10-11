# 方式三 RGB-D 实时视觉跟随

本组负责方式三，以比赛满分要求为验收目标：识别正确、持续跟随、主动绕障、无碰撞、终点进入并停稳。现已连接 Kinect/RGB-D 与真实底盘，独立雷达待确认。2026-10-10 用户确认当前目标改为前面的 Turtlebot，仍使用本车 RGB-D 视觉，不改为方式四的前车地图位姿通信。

**现场运行先看本文末尾“场地测试入口”，验收见 [验收说明](../../docs/method3_acceptance.md)。**
当前已适配前车 Turtlebot：结合 RGB 外观和配准深度识别底盘，使用实时点云地图自主选点、规划与控制。
已有实车移动证据，完整多障碍绕行与移动前车的连续跟随仍待验收。
2026-10-11 新增上层平台深度辅助、遮挡时共享实测支撑去重，以及最新帧处理；
预览绿色框表示确认目标、黄色表示候选、青色表示平台支撑、红色表示拒识候选。
红色拒识仅表示不是跟随目标，该物体仍保留在碰撞地图中。
具体改善幅度和剩余背景/遮挡问题见 [现场诊断记录](../../docs/method3_vision_field_diagnosis_20261011.md)。
[团队接手说明](../../docs/method3-team-handoff.md) 和下方 TonyPi 教程保留早期环境/标定过程；
其中旧模板、间距及速度不代表当前 Turtlebot 场地配置。

## 当前进度

| 阶段 | 状态 | 完成依据 |
|---|---|---|
| 比赛要求和官方 follower 调研 | 已完成 | 原 follower 没有 TonyPi 身份识别或主动绕障 |
| RGB-D 数据诊断与无设备演练 | 真机数据链路和 ROS 构建已验证 | Kinect 实时 RGB/配准深度，隔离 ROS 冒烟 |
| RGB 外观、深度候选和持续关联 | 已适配当前前车 Turtlebot | 同一真实深度支撑区域融合两种候选；跟踪时间占比不等于识别准确率 |
| 跟随控制、主动绕障、终点判定 | 295 项 RGB-D 离线测试通过；完整实车绕障未通过 | 一轮近直线移动约 0.80 m；更新后侧向接近轮累计 0.6692 m，目标有效状态占比 96.18%；收尾轮累计 0.2203 m |
| 全队实机满分验收 | 未进行 | 需真实场地反复试跑与完整记录 |

`rgbd_probe` 检查手工选定的矩形区域（ROI），不是 TonyPi 检测器。`roi_depth_ok` 仅代表区域深度通过设定检查，不证明区域内是 TonyPi，更不证明配准、路径或车体周围安全。节点只发布 JSON 状态和标注图像，不输出速度。

## 第一课 在虚拟机完成数据链路演练

以下按 Ubuntu 16.04 / ROS Kinetic / Python 2.7 编写，先在虚拟机核对：

```bash
lsb_release -a
echo "$ROS_DISTRO"
python --version
```

若不是该环境，先记录实际输出，再调整依赖；不要将 Kinetic 的 cv_bridge 直接用 Windows 或 Python 3 运行。

### 1 安装和构建

将 GitHub 最新代码同步到虚拟机 `~/intelligent_robot_system`：已有仓库可执行 `git pull --ff-only`；也可通过共享文件夹复制完整项目。确认存在 `turtlebot/follow3/rgbd_following/package.xml` 后再构建。

```bash
source /opt/ros/kinetic/setup.bash
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-message-filters \
  ros-kinetic-image-view python-numpy python-opencv python-catkin-pkg
mkdir -p ~/catkin_ws/src
```

确认 `~/catkin_ws/src/rgbd_following` 尚不存在后创建链接；已有同名包时先检查路径，不覆盖：

```bash
ln -s ~/intelligent_robot_system/turtlebot/follow3/rgbd_following \
  ~/catkin_ws/src/rgbd_following
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rospack find rgbd_following
```

### 2 启动无设备演练

终端 A：

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch rgbd_following demo.launch
```

终端 B（每个新终端都需 source ROS 和工作空间）：

```bash
rostopic echo /rgbd_demo/rgbd_probe/status
```

正常应看到 `reason: roi_depth_ok`、`median_m` 约 1.2、`valid_ratio` 约 1.0，始终 `motion_enabled: false`、`target_identified: false`。1.2 米是生成器写入的测试值，不是实测结果。

终端 C：

```bash
rosrun image_view image_view image:=/rgbd_demo/rgbd_probe/preview
```

画面中央绿色矩形的深度为 1.2 米，背景为 2.5 米；框的位置由归一化 ROI `[左, 上, 右, 下]` 控制。

### 3 注入故障并解释输出

每次执行一条，观察终端 B；最后切回 `good`：

```bash
rosparam set /rgbd_demo/source/mode invalid
rosparam set /rgbd_demo/source/mode mixed
rosparam set /rgbd_demo/source/mode stale
rosparam set /rgbd_demo/source/mode frame_mismatch
rosparam set /rgbd_demo/source/mode pause
rosparam set /rgbd_demo/source/mode good
```

| mode | 预期现象 |
|---|---|
| invalid | `insufficient_depth`，无有效深度，不能当作空旷 |
| mixed | `mixed_depths`，前景和背景混在框内，不能盲取距离 |
| stale | `stale_or_future_input`，随后可能伴随 `input_timeout` |
| frame_mismatch | `registered_frame_mismatch`，随后可能伴随 `input_timeout` |
| pause | 最后一次有效输入超过 0.5 秒后 `input_timeout` |
| good | 恢复 `roi_depth_ok` |

以上演练不依赖底盘、相机或 Gazebo，也不能证明真实识别或避障。完成后记录 ROS 版本、构建结果和各故障输出。

## 第二课 拿到设备后测真实数据

只启动一套相机驱动，检查 `rostopic list`。要求同一相机的彩色校正图、配准到彩色相机的深度图，以及对应 CameraInfo；仅修改 frame_id 不能代替配准。诊断器要求三个消息的尺寸、坐标系一致且时间差不超过 0.05 秒，CameraInfo 应持续随图像发布。

下面是常见话题名，必须以实际驱动为准：

```bash
roslaunch rgbd_following probe.launch robot_ns:=tb3 \
  rgb_topic:=/camera/rgb/image_rect_color \
  depth_topic:=/camera/depth_registered/image_rect \
  info_topic:=/camera/rgb/camera_info \
  roi:='[0.4, 0.35, 0.6, 0.65]'
rostopic echo /tb3/rgbd_probe/status
rosrun image_view image_view image:=/tb3/rgbd_probe/preview
```

先让 TonyPi 的躯干覆盖框内区域，再测正面、背面、侧面、抱物行走和左右转弯。记录实际距离、深度中位数、有效比例、P90-P10 深度跨度和遮挡情况。用卷尺对照深度，检查物体轮廓与深度边缘对齐；尺寸和 frame_id 一致并不能证明真正配准。

参数默认值仅用于诊断：深度范围 0.4~4.0 米、有效样本至少 30、比例至少 0.5、P90-P10 跨度不超过 0.25 米，不代表已知 Kinect 实测能力或最终跟随距离。0/NaN/Inf 和超范围深度均不作为有效测量。preview 中断时窗口可能显示最后一帧，必须结合 status 判断新鲜度。

## 完整跟随代码

| 文件 | 功能 |
|---|---|
| `scripts/rgbd_target.py` | 同步 RGB/配准深度/CameraInfo，发布原始时间戳点云；颜色与深度筛选后在 odom 中关联同一目标 |
| `src/rgbd_following_core/vision.py` | HSV 多区间、轮廓、有效深度比例、中位数与深度聚类、物理宽度筛选 |
| `src/rgbd_following_core/tracking.py` | 多帧确认、跳变/歧义拒绝、丢失重确认、实测折线轨迹、全车进入终点判定 |
| `scripts/rgbd_follower.py` | 选择沿目标历史路径后退的跟随点，发送 move_base 目标，检查任务状态/TF/地图/目标新鲜度 |
| `config/navigation.yaml`、`config/costmap.yaml` | 点云驱动的滚动 odom 地图，Navfn + DWA 主动规划绕障；无需 map_server 或 AMCL |
| `scripts/rgbd_velocity_gate.py` | 独立壁钟看门狗、车体扫掠和停止距离检查，默认只发布预览速度 |
| `config/follow.yaml` | 识别、跟踪、间距、速度、终点及实机标定开关 |
| `tools/calibrate_rgbd_target.py`（仓库根目录） | 从照片中紧裁的目标色块生成候选 HSV 配置，包括红色色相跨零处理 |

颜色只是可标定的外观特征，不能直接证明身份。默认绿色用于合成输入，不代表 TonyPi 的真实颜色。明天应选择本体上足够大、稳定且区别于背景的色区，或裁判允许的彩色标识；若本体没有足够区分度或侧身时特征消失，需要调整标识/相机布置或替换识别器，不能靠放宽阈值宣称满分条件已经满足。同色目标靠得很近仍可能无法区分，程序在歧义时停车。

### 1 明天先做颜色与深度标定

先使用前述 probe 确认输入，再保存不同角度、步态和光照的彩色图及配准深度。示例（ROI 数字为照片中的像素坐标，应替换为实际裁剪）：

```bash
python3 tools/calibrate_rgbd_target.py tonypi.png \
  --roi 130 90 190 150 --output /tmp/tonypi-color-candidate.yaml
```

工具不会覆盖已有输出，会保留三个实机标定标志为 false。它只估计颜色，深度范围、物理宽度、地面高度、车体半径和制动能力仍要实测。每个配置应在正面/背面/侧面与干扰物图像中复核；不要只测一张正面照片。

该工具可在 Windows 的 `.venv/Scripts/python.exe` 下使用；若在虚拟机运行 `python3`，需另行确认该解释器能导入 cv2、numpy、yaml，ROS Python 2 已安装这些库不代表 Python 3 也有。

### 2 构建并启动影子模式

在原有依赖之外安装课程对应的导航包，再 catkin_make：

```bash
sudo apt-get install ros-kinetic-navigation ros-kinetic-tf2-ros
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch rgbd_following follow.launch execute:=false \
  settings:=/tmp/tonypi-color-candidate.yaml
```

底盘驱动/相机需单独启动。默认针对一台课程环境车辆：TF 为 `odom`、`base_footprint`，传感器话题在 `/camera/...`，里程计 `/odom`，最终底盘输入 `/cmd_vel_mux/input/navi`。如实际名称不同，在 launch 参数中覆盖。不要同时启动旧 follower、旧导航或其他速度控制源。

影子模式计算候选速度但只发布 `/tb3/cmd_vel_preview`，不创建底盘输入发布者。查看：

```bash
rostopic echo /tb3/perception_status
rostopic echo /tb3/follow_status
rostopic echo /tb3/gate_status
rosrun image_view image_view image:=/tb3/target_preview
```

`perception_status: tracking` 表示当前满足配置的多帧关联条件，不等于已经完成识别可靠性验收。启动时若实测轨迹不足 `follow_distance`，但目标距离足够，控制器会根据新鲜目标位置生成保持间距的导航目标，状态为 `following_live_standoff`；轨迹足够后恢复沿实测轨迹跟随。此过程不补造历史路径，仍需通过地图、身份、里程计和速度门控检查。距离不足时原地等待，不生成倒车目标。

所有跟随模式都需要持续任务心跳。与负责 TonyPi 的小组协商接入 `/competition/mission`，可复用已有 UDP 心跳桥。进行独立影子测试时，可把 `mission_topic:=/tb3/test_mission` 传给 launch，在另一终端持续发布私有测试信号：

```bash
rostopic pub -r 5 /tb3/test_mission std_msgs/String "data: 'carrying'"
```

### 有界实车调试

`tools/run_rgbd_bounded_trial.py` 是人工在旁监督的调试转发器，会向明确指定的真实底盘输入发送速度。它不代表比赛验收完成，也不修改三项标定标志。使用前须核实车体空间、真实底盘命令超时和人工停机方式；`disabled_velocity_input` 这样的命名不代表话题与电机断开，要检查实际订阅者。

工具只转发方式三影子门控产生的命令，要求目标、门控、里程计、底盘传感器和外部运行许可持续新鲜；碰撞、悬空、过流、失联、未知区域或速度超限均阻止或锁存停车。它检查输出独占及预期上游发布者，不生成目标、速度或识别有效信号。另需在测试命名空间持续发布任务心跳。

```bash
# 先把 follow.launch 的 robot_ns 设为 method3_motion，execute 保持 false，
# mission_topic 和 estop_topic 分别指向该命名空间的 test_mission/test_estop。
# 运行许可与急停须由现场操作进程持续发布，停止许可即停止本次测试。
python tools/run_rgbd_bounded_trial.py --namespace /method3_motion \
  --output /实际底盘速度输入
```

默认最高 0.05 m/s、0.15 rad/s，最长 4 秒，累计路程达到 0.06 m 提前停车，并保留角度和制动余量。0.06 m 是停止触发点，不是未经实测的最终停车距离保证；整个进程或虚拟机失效时仍依赖底盘超时。最终 JSON 区分请求、实际转发、实测速度，并记录编码器变化和 PWM；收到非零命令不等于车轮已经运动。

前视相机无法一次观测起步区域的所有侧方栅格。若完整目标足迹碰到未知区域，应在赛前补充真实观测，不能用清空地图或把未知设为自由来放行。极短的调试目标也需匹配地图分辨率、位置容差和预测时域；临时试验参数与正式跟随配置应分开记录。

停止该发布后应回到 `mission_stopped`。测试 done 前先停止 carrying 发布，然后将 data 改为 done；不要同时发两种状态。私有测试信号只用于隔离调试，正式比赛切回真实任务源。

### 3 通过标定后执行

记录实际颜色识别可靠性、RGB-D 配准、相机 TF、高度过滤、车体半径、制动距离、侧面/低障碍覆盖和完整终点区域。`follow.yaml` 中填入凸多边形 `finish_polygon`，坐标属于本车本次开机的 odom；初始化摆位或 odom 重置后必须重标。

确认完成后再将 `identity_calibrated`、`depth_coverage_calibrated`、`hardware_calibrated` 设为 true。调整限速/半径时，要同步修改 `navigation.yaml` 和 `costmap.yaml`：规划器不能快于门控允许速度；圆形半径应覆盖车体和突出物，不能为穿窄通道而缩小。

```bash
roslaunch rgbd_following follow.launch execute:=true \
  settings:=/path/to/validated-follow.yaml
```

多机时明确指定每车自己的帧、输入和输出，例如：

```bash
roslaunch rgbd_following follow.launch execute:=false robot_ns:=tb3 \
  odom_frame:=tb3/odom base_frame:=tb3/base_footprint odom_topic:=/tb3/odom \
  rgb_topic:=/tb3/camera/rgb/image_rect_color \
  depth_topic:=/tb3/camera/depth_registered/image_rect \
  info_topic:=/tb3/camera/rgb/camera_info \
  cmd_vel_topic:=/tb3/cmd_vel_mux/input/navi
```

这部分只依赖本车里程计和传感器 TF，无需其他组共享地图原点。跟随目标始终来自自身 RGB-D 检测，不能把另一辆车的位姿当作方式三的识别结果。

### 4 停车、绕障与终点的边界

- 目标丢失立即撤销导航、关闭许可，原地多帧重新确认。强外观模板匹配支持在有上限的运动范围内重识别；无模板时仍只允许近距离关联。两个合理候选时停车，超 60 秒锁定故障。
- ABORTED/REJECTED/LOST、目标大跳变或 odom 大跳变会锁定停车；没有自动旋转找目标/清图恢复。重新定位后重启清除旧轨迹。
- 导航拒绝未知区域；门控检查车体扫过区域和停止距离。没有可用路径时停车，不能保证任意障碍布局都有绕行解。
- Kinect 仍有视野、近距离和遮挡盲区。VoxelLayer 保留高度占据并做三维射线清除，但投影给规划器的地图仍是二维，不能证明所有高度均已看见；4 cm 以下障碍需实测。若覆盖无法满足要求，补充传感器或调整布置，不能关掉未知区域检查来强行行走。
- done 后必须同时满足目标停稳、到达跟随点、整车进入 finish_polygon、里程计线/角速度低于阈值，并持续 2 秒才 finished。到达之前不累计停稳时间。`finish_zone_not_reached` 表示需修正终点布局/领车停车位置，不能当作完赛。
- ROS `/competition/estop` 接入门控，持续 true 会停车；它不控制 TonyPi。所有速度控制源必须统一，禁止其他节点绕过门控直接发底盘速度。

与其他组联合验证遮挡、相互碰撞与终点容纳空间。单元测试或单车跑通都不能代替全队满分验收。

## 四项优化的使用与验收

### 1. 多视角身份模板与遮挡后恢复

颜色轮廓作为候选种子；在种子周围同一深度的区域提取四分区 HSV 直方图，结合种子宽高比和米制宽度，与人工登记的多视角模板比较。这样能利用相邻的其他机身颜色，并支持多种侧身/抱物外观。模板不在线自学习，避免误识别后不断学习干扰物。它不是通用 TonyPi 分类器，外观相同的物体仍可能混淆。

先录制真实注册 RGB-D（见下文），在已 source ROS 的仓库根目录执行：

```bash
python tools/extract_rgbd_samples.py /data/enrollment.bag --output /data/identity_samples
# 编辑 manifest.json：每个 roi=[x,y,width,height] 框住唯一目标颜色种子，
# label 标记 front/back/left/right/carrying 等真实视角。剔除遮挡不清的登记帧。
python tools/enroll_rgbd_identity.py /data/identity_samples/manifest.json \
  --config /data/color-calibrated.local.yaml --output /data/identity.local.yaml
```

登记程序会重新关闭三项标定开关；**实机执行还要求 references 非空**。用另一段不同光照、角度、同色干扰物的视频导出验证样本；没有 TonyPi 的帧将 `roi` 设为 `null`，有目标的帧填写实际 ROI。

```bash
python tools/enroll_rgbd_identity.py /data/heldout/manifest.json \
  --config /data/identity.local.yaml --evaluate --output /data/identity-evaluation.json
```

报告候选层面的 precision/recall、误检数与歧义帧数；不能用登记帧冒充独立验证。若左右侧没有种子颜色，要补相应 HSV 色带和模板，或调整经裁判允许的特征布置。当前不做部分遮挡形状补全，严重遮挡时停车。

强模板匹配的丢失恢复需默认连续 6 帧，搜索范围随失联时间按最大目标速度增长，上限 1.5 m；没有强匹配时不放宽原 0.45 m 门限。丢失期间不移动找目标。恢复后清空断裂的历史轨迹，先对实时目标生成带间距的导航点，经 move_base 和门控检查；不会把遮挡期间的未知运动用一条直线伪装成已观测轨迹。新轨迹长度足够后恢复沿轨迹跟随。

### 2. 绕障时的局部视野约束

门控订阅实时 CameraInfo、目标三维位置和相机安装 TF，预测候选速度下未来 1 秒的目标投影。默认保留画面四周 12% 余量；原速度不满足时，以相同比例缩小线/角速度为 0.6、0.3 倍，并重新检查碰撞。还检查实际里程计速度下的保守停止视野路径，不能假定车已瞬间降速。

这是局部速度约束，不是能预测遮挡物或保证全局可见性的规划器。目标位置在短时预测中保持不变，快速横移仍可能丢失；边缘余量、相机倾角和预测时长必须实测。无可行候选、相机信息/目标/TF 过期时输出零，已丢失时不自动旋转。实机验收应观察左转、右转和绕障时的目标画面位置及 `gate_status`，避免反复停走。

### 3. 实际速度制动与三维障碍清除

follower 和 gate 均检查 odom 帧名、child_frame_id、时间戳、有限速度与新鲜度。默认 odom 超过 0.3 秒失效就停车。停止包络半径为：

`车体半径 + max(实测速度,候选速度) × 反应时间 + max(实测速度,候选速度)² / (2 × 制动减速度)`

采用全方向圆包络覆盖不同角/线减速度产生的曲率变化，因此在窄通道可能更保守。假设反应期间速度不超过上述上界、配置减速度不高于实际可保证的制动能力；需在实际地面、电量、载荷下测量。下发零速度也不代表已停稳。

costmap 使用 ROS VoxelLayer，默认高度 0～1.6 m、16 层、每层 0.1 m，标记障碍的最高高度为 1.5 m。Kinetic 会把负的 `origin_z` 钳制到 0，配置现显式使用 0。地面射线只清除三维经过的体素，保留其他高度已标记的障碍。相机原点必须位于高度范围内。`unknown_threshold: 15` 只要求部分体素被观测就可将未占据列标为空闲，**不是整列全高度覆盖证明**；必须配合低障碍、侧方、近场和多车遮挡测试。不要通过缩车体半径或放开未知区域来解决卡住。

标记与清障使用独立观测：`obstacle_cloud` 保留原有识别/碰撞深度范围，`clearing_cloud` 使用 `perception.clearing_max_depth` 和 `clearing_stride`。示例清障范围上限为 4.5 m，需核查实际相机在该距离的有效性；旧标定文件若缺少这些字段，仍沿用其原深度上限，不会静默扩大量程。清障源不标记障碍，允许较高/较低的真实端点进入 VoxelLayer，再由三维射线与地图边界裁剪。不能先按车体高度过滤掉背景端点。两路使用原始采样时间戳和相机坐标系；输入过期不发布，远背景清障不放宽运动门控对近场深度覆盖的要求。

近场覆盖不足时先发布 `target_valid=false`，同时仍用当前帧真实的稀疏或空标记云替换旧观测，防止 `observation_persistence: 0` 保留的上一帧障碍被反复标记。空标记云不会清除已有体素，清除仍需要有效背景射线；不能通过持续保留旧标记观测实现故障保护。

NaN、Inf、零、超出设定量程的像素均不产生清除射线。现场保存帧的真实 VoxelLayer 回放中，上限 4 m 时 9 个旧体素全部残留；4.5 m 时自动清除 4 个，仍有 5 个体素分布在 2 列。这说明过滤问题已改善，但缺测方向仍需改变视点或提供可测背景，不能用定时清空、TTL 置空或地面回波清整列来伪造通过。该回放使用保存画面和合成旧状态，不能代表实测清障耗时。

```bash
# 独立 master 的真实 VoxelLayer 回归；不使用 --reuse-master
rostest rgbd_following clearing_observations.test
# 保存现场需包含 metadata.json、depth.npy、voxels.npy 和 comparison.json
python tools/replay_clearing_snapshot.py --help
```

**起步也需要周围覆盖。** 前向相机冷启动时，车体侧后方尚未观测，DWA 的完整足迹可能找不到合法轨迹。比赛开始前应按裁判允许的准备方式取得起点周围的真实深度观测，或使用经确认可用的补充传感器；回到起始朝向后标定终点区域，保留同一 odom。检查 RViz 中车体与第一段停车包络都在已观测空间内。程序不会把未知区域直接填为空闲，也不会在比赛中因丢失目标而自动扫描。ROS 冒烟测试通过合成传感器的赛前环视观测初始化覆盖，该准备条件不代表实机已完成。

### 4. 录包、隔离回放和完整赛道验收

启动跟随节点后另开终端记录。输出目录应已存在，JSONL 不覆盖旧文件；实际执行时把 command_topic 改成实际底盘输入，默认记录影子速度。

```bash
roslaunch rgbd_following record.launch scenario:=left_right_turns \
  output:=/data/left-right-01.jsonl bag_prefix:=/data/left-right-01 \
  settings:=/data/identity.local.yaml
```

多车时显式传入本车的 odom/base 帧、相机、里程计、mission、estop 和命令话题参数。录包包含注册 RGB-D、CameraInfo、TF、odom、任务状态和诊断。原始 RGB-D 数据量较大，事先检查剩余磁盘。

在没有硬件驱动的独立 ROS master 中回放：

```bash
# 终端一
ROS_MASTER_URI=http://localhost:11321 roscore -p 11321
# 终端二；启动后按空格开始，以真实时间速率回放
ROS_MASTER_URI=http://localhost:11321 roslaunch rgbd_following replay.launch \
  bag:=/data/left-right-01_XXXX.bag settings:=/data/identity.local.yaml
```

回放固定 `execute=false`，只重放传感器/TF/任务输入，不播放录包中的旧速度或旧识别结果。暂停触发壁钟看门狗是预期行为。此回放用已记录的本车运动评估感知与控制决策，不能证明新控制器驱动下的闭环路径；闭环和真实绕障仍需实机验收。

每次试跑只对应一个 scenario：`left_right_turns`、`occlusion`、`same_color`、`crossing`、`obstacles`、`team_finish`。建议每类至少连续完成三次，并保存完整视频、日志及每次失败记录。人工核查零碰撞、身份正确、无干预、整车终点停稳；最后一类额外核查全队完赛。

按试跑时间顺序整理 `trials.json`（每次独立记录一项，保留失败项，不可重复计入同一个日志；scenario_exercised 需视频/现场确认实际完成了该场景）：

```json
{"trials":[{"scenario":"left_right_turns","log":"left-right-01.jsonl",
  "verdict":{"scenario_exercised":true,"collision_free":true,"correct_identity":true,
    "no_intervention":true,"robot_stopped_in_finish":true}}]}
```

```bash
python tools/evaluate_rgbd_trials.py /data/trials.json --repeats 3 --output /data/report.json
```

报告可见时间占比、重识别耗时、最长失联、欧氏距离误差 P95、地图估算最小间隙、传感器覆盖率、各场景成功率及末尾连续成功次数；每类最后连续三次通过才满足默认整套验收。曲线路径上的欧氏间距与沿轨迹间距不同；地图间隙不等于真实碰撞检测。缺少人工结论、未测得终点停车、日志断档或传感器证据不足均不能自动通过；报告不换算比赛分数。

真实 Kinetic 集成检查可在有 Docker 的机器上执行：

```bash
docker build -f tools/ros-kinetic.Dockerfile -t rgbd-kinetic-check turtlebot/follow3/rgbd_following
docker run --rm rgbd-kinetic-check
```

镜像固定 Kinetic/Xenial 基础镜像摘要并使用 ROS 最终快照依赖；执行 catkin 构建和真实 ROS/cv_bridge/TF/VoxelLayer/DWA 冒烟测试，使用合成 RGB-D 和简化运动学，检查影子导航与丢失停车。它不是 Gazebo 或完整赛道验证。

## 离线验证与技术依据

当前 Windows 独立虚拟环境中全项目 129 项测试通过，无跳过；新增无色标检测的 11 项测试也已在实机连接的 Kinetic/Python 2.7 虚拟机通过，catkin 构建通过。清障回归已在隔离 ROS master 上通过真实 Kinetic VoxelLayer 验证，现场快照重放仍有缺测残留，不能视为整场动态验收通过。此前 Kinetic/Xenial 基础容器下 52 项核心测试通过，4 项仅适用于 Python 3 的报告测试跳过。完整 ROS/navigation 集成与硬件验收状态见 [验证记录](../../docs/validation.md)。

```bash
python3 tools/validate_project.py
python -m unittest discover -s tests -p 'test_rgbd*.py' -v
```

深度和控制核心测试不依赖 ROS、OpenCV 或 NumPy，可用 Python 2.7/3；视觉流水线测试需 OpenCV/NumPy/PyYAML；ROS 适配层替身测试使用 Python 3。Windows 可使用仓库 `.venv/Scripts/python.exe`，虚拟机的 ROS 节点仍使用其系统 Python 2.7。真实 ROS 构建、move_base 避障、图像显示和硬件性能仍需虚拟机/设备验证。

接口核对来源：[ROS 深度图单位规范](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst)、[Kinetic 消息同步器](https://github.com/ros/ros_comm/blob/kinetic-devel/utilities/message_filters/src/message_filters/__init__.py)、[Kinetic cv_bridge](https://github.com/ros-perception/vision_opencv/blob/kinetic/cv_bridge/python/cv_bridge/core.py)。

## 不加色标的现场标定

### 跟随前车 Turtlebot 的初步适配

Turtlebot 的底盘与层板可能在深度图中分离，不能直接复用 TonyPi 的整体宽高范围和模板。
`perception.body.max_support_height_m` 可选地限制识别使用的离地高度带，
例如现场以 0.17 m 上界登记可见底盘区域，再结合实际宽高与独立身份模板筛选。
该高度带只作用于识别，不裁剪避障点云；未配置时保留原行为。
参数必须针对实车采样验证，黑色底盘的无效深度、相似车辆和人的脚部仍可能造成漏检或歧义。

本次前车初步适配：人工确认底盘 ROI，3 帧登记、另 5 帧静止留出均为唯一目标，
随后 6 秒实时预览中 24/24 帧为一个候选。它尚未通过无目标、转向或运动验收，
不能据此开启完整跟随。当前参考模板保留在现场数据目录，所有标定标志仍为 false。

黑白 TonyPi 的正面白色胸板在背面不可见，不能直接沿用绿色示例或胸板模板。
可在本地配置中设置 `perception.proposal_mode: depth_body`，清空旧 `identity.references`，
使用已有 `tools/enroll_rgbd_identity.py` 重新登记正面、背面、左右侧和抱物状态的完整身体样本。
manifest 的 ROI 此时选择整个身体候选中心；`body` 中的宽高范围需按实际设备检查。
此模式不依赖 `hsv_bands` 分割，但保留该字段以兼容默认配置结构。

算法从画面下半部深度拟合可见地面，在地面上方按重叠深度区间提出身体候选，
结合物理尺寸、近地支撑、空间明暗/色彩分布和轮廓匹配固定模板。
灰色区域使用亮度特征，避免仅用不稳定的灰色色相。
没有可靠地面时不输出候选；多个不同物体不合并为一个目标，跟踪器继续按歧义停车。
地面剔除只用于识别，避障点云仍保留原有全部深度观测。
该方法假设相机基本直立、能看到平坦地面；台阶、地面被遮挡、裁切身体和相似外观干扰均须另测。
增加模板不会自动解决所有遮挡，所有实机校准标志仍必须保持 false，直到对应验收完成。

现场 Kinect 驱动实际配准深度话题为 `/camera/depth_registered/hw_registered/image_rect`，
不同驱动应先核实话题，不能直接套默认名。
无需底盘 TF 的独立预览只发布图像和诊断，不发布目标、导航或速度：

```bash
roslaunch rgbd_following identity_preview.launch settings:=/absolute/path/body.local.yaml
rosrun image_view image_view image:=/rgbd_identity_preview/preview
```

窗口显示 `UNVALIDATED - MOTION DISABLED`，框为外观候选，不代表已通过身份验收。
输入过期时显示空白错误画面，防止把旧图当实时结果。
首次现场仅采集了正、背面各一组样本；背面实时静止观察 10 秒，共 47 帧均有一个候选，
检测耗时最高约 20 ms。这是单场景初步证据，不是整场成功率或满分证明。
左右侧、抱物、无目标、同色干扰、遮挡、相机外参及真实制动仍未完成。
采集的原始图像、深度和未验证模板保存在操作机器上，不提交到仓库。

增加球、挡板、另一台底盘和椅子后，旧地面占比门槛导致 24 帧全部漏检。
地面估计现接受占比至少 25% 的拟合支持，但同时要求证据跨越画面宽度、
靠近底部并覆盖多个深度，且进行三轮平面精修；狭窄地面条带仍拒绝。
保留原有外观模板与识别阈值，未把障碍区域加入目标模板。
修复后的同场景 12 秒静止观察共 51 帧：48 帧有单个候选，3 帧未检出，
检测耗时最高约 78 ms；这仍有漏检，不能据此宣称复杂赛道识别或绕障验收通过。
另外本次测试前 Kinect 曾停止出帧，重启驱动后恢复，USB 长时间稳定性仍需排查。

### 近车盲区的短距离调试

`follow.launch` 的 `commissioning_clearance` 默认关闭。现场人员检查指定区域后，调试启动时可设置 `commissioning_clearance:=true`，再执行 `tools/publish_commissioning_clearance.py --confirm-clearance --namespace /method3_motion`。该工具不发送速度；默认记录底盘原点前 0.60 m、后 0.50 m、左右各 0.50 m 的地面矩形。区域固定在 odom 中，不随小车移动，默认 10 秒自动失效；障碍层发现的障碍不会被抹除。它只用于已检查场地的短距离联调，不能替代比赛运行时的近场传感器覆盖。实际运动仍使用 `run_rgbd_bounded_trial.py` 的独立限幅测试。

目标终点暂时不可通行时，初始实时跟随现在可选择已观测通行区域内的中间目标；完整车体扫过的路径仍逐段检查。历史目标轨迹不被直线截弯，终点完成条件也不变。
## 2026-10-11 场地测试入口（尚未完成全程绕障验收）

当前部署：Ubuntu 16.04 / ROS Kinetic，工作空间 `~/codex_method3_ws`，
运行配置 `~/codex_method3_runtime/turtlebot_follow_current.launch`。
该配置使用本车 RGB-D 识别前方 Turtlebot，期望跟随距离 0.6 m，
正常停车点可缩到 0.5 m，0.4 m 是底盘原点到前车识别表面点的停止前进阈值。
若这个距离范围被障碍占用，允许暂时选择更远的可达绕障中间点，之后再收敛到跟随距离。

在 Ubuntu 执行（入口自动补启动 ROS master）：

```bash
bash ~/codex_method3_motion_20261010/tools/start_method3_field.sh
```

入口自动启动缺失的相机、底盘及跟随节点，打开识别预览和 RViz 点云窗口，记录状态和 rosbag，
运行最长 300 秒、累计行程最多 5 米的场地测试。默认最大下发速度 0.10 m/s、0.30 rad/s。
可用 `--max-speed 0.05 --max-turn 0.15` 降速；规划器、速度门控和转发器使用同一组上限。
Ctrl+C 停止运动和本轮记录，默认保留相机、底盘、感知和可视化，便于下一轮测试；
`--shutdown-stack` 还会关闭本轮启动的这些进程。`--view-only` 仅启动数据链路和窗口，不启动运动。
入口用 `flock --close` 保证同一时间只有一个由该入口启动的会话，重复执行会提示已有会话，
不重启正在运行的节点；应先在原终端 Ctrl+C，再开始下一轮。保留的驱动和窗口不会继续持有会话锁。
`--help` 参数已修复，可直接查看选项；虚拟机上的 shell 语法和帮助入口已验证。
底盘实际输入为 `/codex_diagnostics/disabled_velocity_input`，其名称不表示禁用。
必须保留 `sensors.launch` 的对应 remap；不能换为另一个底盘启动文件后直接运行。

场地入口默认使用 `--allow-unobserved`：未观测栅格按可规划区域处理，
这是当前场地测试的规划假设，不代表相机已证明该区域无障碍；实时点云障碍仍阻挡路径。
该模式关闭临时人工净空插件，不需要重复登记或清空地图。
若要使用严格未知区域策略，应直接调用 `run_rgbd_field_session.py --start-stack`，不加该选项。
此入口尚未通过完整比赛赛道验收。
发生目标丢失或路径阻挡时发送零速度，等待新鲜且通过检查的目标/规划输出；
持续暂停60秒结束。场地模式下短暂的目标、指令或门控消息超时同样先保持零速度，
所有必要消息恢复新鲜后才恢复运动。里程计、底盘传感器、输出节点唯一性等异常直接终止。
field 模式的实测角速度超过命令上限加 0.03 rad/s 时也立即输出零；连续新里程计超限 0.10 秒后终止。
恢复需连续新里程计在命令上限内保持 0.10 秒，不接受重复旧消息，暂停期间预算继续累计。
当前 0.30 rad/s 命令上限对应 0.33 rad/s 暂停阈值和 0.40 rad/s 严重异常终止阈值；
后者仅决定立即结束会话，并非允许小车带着超速继续运行。线速/横向超限仍即时终止，严格短程 commissioning 行为不变。

场地配置启用 `detour_max_extra_distance: 0.6`、`navigation_retry_delay: 0.8`。
绕障点经过全车轮廓和地图连通性检查；普通导航 ABORTED/REJECTED 后等待新地图再换点重试，
不再永久锁死。LOST、目标/里程计跳变和其他终止条件仍保留。
控制器允许零线速度转向，使用四档动态窗口线速度采样，保留中间速度的曲线轨迹。
正常距离停车点与更远的绕行点一起评分，代价包含可达路径长度、软障碍代价、间距误差，
以及实际重建路径前 0.25 m 方向与前车方向的夹角，允许先向侧方或暂时远离前车；
场地模式从真实 Dijkstra 路径提取约 0.30 m 前瞻子目标，再交给 Navfn/DWA 执行，
避免只将最终停车点交给局部控制；路径代价权重仍为 0.25。
短期排除已经到达或失败的中间点。视野门控连续拒绝同一目标后重新规划，不能靠反复同一转向脱困。
可用绕行点最长保持 8 秒，避免深度微小波动导致左右目标反复切换；前车明显移动、终点新被障碍占据、
导航失败或持续视野拒绝仍会触发调整。这个保持时间不冻结实时点云、局部规划和碰撞检查。
对目标尚未到达且门控持续限速的情况，控制器结合真实平移与转角判断是否没有进展，
触发 `navigation_retry_no_progress` 换点重试；到达跟随间距后的正常停车不会被当作卡住。
前车仍在画面边缘、DWA 平移弧线会丢失目标时，Gate 可独立选择能改善视野且通过碰撞/制动检查的原地回正，
不必等 DWA 恰好提出零线速度候选。丢失目标后的停留行为保持。
本车根据用户测量约 36 cm × 30.5 cm 使用 0.25 m 圆形包络（含余量），不另叠加 padding；
其他车沿用各自运行 YAML 的 `footprint_radius`，不能仅因想加速而缩小车体尺寸。
目标容差为 0.05 m，避免原先 8 mm 容差与持续前进限制共同导致难以到达停车点。

日志目录会打印为 `SESSION_LOG`，位于 `~/codex_method3_runtime/field_sessions/`。
其中 `motion.log` 含运动量和结束原因，`status.jsonl` 含识别/规划/速度门控状态变化，
`telemetry.bag` 用于回放。可用 `--duration 12 --distance .15` 做短轮测试。
每次命令重建跟随节点，清除上一轮终止锁存；保持正常的底盘和相机驱动。
启动同时检查 RGB 与配准深度，避免只有深度却误判整套相机可用。
任一路停止更新时，报告具体缺失的图像流；自动恢复会重建本相机 namespace 内的完整 nodelet 链，
包括可能残留但不再发布 RGB 的 rectifier loader。`perception_status` 记录经过同步检查的
`rgb_stamp`、`depth_stamp`、`info_stamp` 和处理耗时，录包可据此检查输入新鲜度。
验收方法见 [method3_acceptance.md](../../docs/method3_acceptance.md)：分别验证直行、左右绕障、多障碍、遮挡及穿行。

识别同时评估 RGB 底盘模板和深度形状，不因深度候选已经存在就跳过视觉方案。
只有空间位置、深度和真实测得的支撑像素一致且关联唯一时才融合；不同物体或歧义候选保留给跟踪器判断。
黑色底盘的深度存在空洞时，RGB 可稳定框中心，测距仍来自真实配准深度，不按模板大小推算距离。
RGB 模板候选的 NMS 排序同时考虑实测物理宽度与登记宽度的一致性，避免半个底盘的局部高分框
抑制完整底盘框。现场问题中的 64 像素框测得约 0.159 m，完整 140 像素框约 0.352 m，
登记宽度为 0.30 m；原 NMS 重叠阈值和身份判定阈值没有放宽。
障碍点云独立保留其他物体，不把前车识别模板用于判断哪些物体需要避让。

自主避障采用滚动点云代价地图、Navfn 全局路径和 DWA 局部速度采样，
可达停车/绕行目标选择使用考虑车体轮廓的连通搜索和 Dijkstra 路径代价。
算法处理当前地图中的障碍布局，没有锥桶固定坐标或逐个障碍写死的转向指令；当前方案不依赖强化学习训练。
这说明代码的规划方式，完整泛化能力仍需通过不同布置实测。

较早现场记录 `20261011_174407_35335`：45 秒累计移动 **0.13518797 m**，
累计转角 **0.426733 rad**，最高实测线/角速度 **0.05544 m/s / 0.21537 rad/s**，
以 `duration_limit` 结束。`20261011_173720` 开头的前一轮中，跟踪状态时间占比约 **98.696%**；
该指标不是人工标注的识别准确率，也不能证明绕障成功。
随后 `20261011_174811_37500` 净位移 **0.798438 m**、累计里程 **0.800125 m**，
记录的目标有效状态 **133/133** 为 true；轨迹接近直线，不能记为已绕过锥桶。
随后 `20261011_175543` 开头试跑：60 秒累计 **0.36125477 m**、转角 **0.69516464 rad**，
实测峰值 **0.07676288 m/s / 0.31922072 rad/s**，因时限结束；后半程 `ambiguous` / `reconfirm` 反复暂停。
该轮 `angular_overspeed_pauses=0`，没有实证触发角速度暂停/恢复；无进展换路及自主回正的完整效果也需对应实测，
不能仅凭代码和离线测试记为现场通过。
更新后的 `20261011_180259` 开头试跑以 `motion.log` 记录累计 **0.6692445 m**，
净位移约 **0.6137 m**、向右位移约 **0.203 m**，跟随距离约 **1.437 → 0.866 m**。
目标有效状态占比由对比轮次的 **38.01%** 提高到 **96.18%**，最长丢失时间由 **13.61 秒**降到 **0.603 秒**；
这是程序状态的轮次对比，不是人工标注准确率。地图推算最小间隙约 **0.019 m**，没有实物测距证据。
该轮角速度软超限暂停 **1 次**后恢复；bag 累计约 0.672 m 的采样差异不替换 `motion.log` 的正式行程记录。
最新 `20261011_180505` 开头收尾轮运行 **20 秒**，累计 **0.220309 m**，
实测峰值 **0.0724983 m/s / 0.379958 rad/s**，软超限暂停 **1 次**后恢复，按时限结束。
收尾静态画面显示 `tracking` 和一个完整 **224 × 68 像素**目标框，深度约 **0.708 m**；
该深度不是直接换算后的底盘跟随距离。机器人结束后停稳。
这些记录证明当前布置下能持续接近前车并出现侧向运动，尚不足以认定完整绕障通过。
最新已执行的 RGB-D 离线回归为 **257 项通过**，项目校验覆盖 **71 个 Python 文件及结构检查**，并不替代闭环实车验收。
当前没有完整驶过障碍、持续跟上移动前车及终点停稳的整套通过记录。
