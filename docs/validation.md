# 验证记录

## 2026-10-10 方式三移动调试准备与本轮验证

本轮修复了以下控制边界：处理已膨胀代价地图时避免重复计算车体半径；目标仍在画面边缘可见时允许通过碰撞检查的回正；重捕获时分开计算连续强匹配与普通匹配，避免范围外候选干扰有效候选；没有历史轨迹的首次起步使用当前目标与跟随间距生成初始目标点。以上源码修复不代表完整赛道已经通过。

Windows 全项目 **179 项测试全部通过**，**58 个 Python 文件静态检查通过**。Ubuntu 16.04 / ROS Kinetic 的真实 catkin 构建通过；独立 ROS master 下 `navigation_smoke.test` 结果为 **1 项通过、0 errors、0 failures，耗时 8.971 秒**。虚拟机报告路径：`/home/haiden/.ros/test_results/method3_motion_smoke_20261010/rgbd_following/rosunit-navigation_smoke.xml`。该测试验证隔离环境中的导航链路，不等于本轮实车已经移动或完成绕障。

当前现场底盘串口设备序列号为 **AL03EQZD**，已建立 `tools/run_rgbd_bounded_trial.py` 的单次限幅调试链路：接收真实方式三影子门控输出，最高 **0.05 m/s**，默认累计里程达到 **0.06 m 时提前发出停车命令**，本轮用户授权的总位移上限为 **0.10 m**。停车后持续发送零速度并记录 **0.5 秒尾段**；诊断保留请求/实际转发/实测速度峰值、左右编码器首末值与差值、PWM 峰值。6 cm 是提前停车触发点，最终停车距离仍取决于实测制动、通信与底盘响应，尚不能保证由该阈值直接得到。`hardware_calibrated`、`identity_calibrated`、`depth_coverage_calibrated` **均保持 false**，未将局部调试视为完整标定验收。

初始两轮限幅试验输出为纯角速度，约 **0.045–0.05 rad/s**，线速度为零；观察到非零 PWM，但左右编码器没有变化。当前证据只确认命令到达和驱动响应，**尚未完成真实正向移动测试**。这两轮不能记为跟随起步成功；真实直行、回正转动、丢失停车、制动距离及完整障碍赛道仍待现场结果补充。

后续用户明确要求直接启动底盘后，进行了独立于自主规划的限时直行检查：指令上限 0.05 m/s、持续不超过 1 秒，4 cm 累计里程提前停止，碰撞/悬空/过流、传感器新鲜度、独占速度发布者及墙钟看门狗仍生效。首轮临时脚本因在查询 ROS graph 前读取时间，导致回调的新时间戳被误判为过期；已将时间读取移到锁内、查询之后修正，不能把该轮 `stale_sensor` 当成通信故障证据。修正后的记录 `motion_20261010/direct_trial_1791636357.json`：左右编码器增量 226/258，里程计累计 **0.020726 m**，实测峰值线速度 **0.034117 m/s**，PWM 峰值 20/20，按时限结束并连续发送 1 秒零速度。结束后硬件速度话题无发布者。该结果证明底盘已发生短距离移动，**不代表方式三自主跟随、未知区域覆盖或绕障通过**。

后续更长的独立直行测试 `motion_20261010/direct_long_trial_1791636621.json`：最高指令 0.05 m/s、最长 4 秒、15 cm 提前停止阈值，累计里程 **0.120006 m**，左右编码器增量 1366/1438，峰值实测线速度 **0.042646 m/s**，按时限结束后持续发送零速度；结束后硬件速度话题无发布者。

当前任务随后确认为 **方式三视觉跟随前面的 Turtlebot**，不是方式四通信跟随。此前 TonyPi 样本、模板和运动记录保留作为历史证据，前车 Turtlebot 的检测及身份模板适配尚未完成。配置/样本移交方式见 [团队接手说明](method3-team-handoff.md)。

## 已进行的离线检查

方式三前车 Turtlebot 识别初步适配：新增可选 `max_support_height_m`，仅在地面模型上的识别候选中选取下方底盘，保留完整避障深度。新增高度带边界验证及上方结构排除/原深度不变测试，全项目 **181 项通过**，58 个 Python 文件静态检查通过。现场数据目录 `turtlebot_capture_1791687781`：人工 ROI `[350,285,160,90]` 登记 3 帧，另外 5 帧均唯一识别；6 秒实时预览中 24/24 帧各一个候选，匹配分数约 0.219–0.312。尚未做无目标、跨视角、遮挡、移动和避障验收。重新连接的底盘为 AI06FBJN，左保险杠位持续为 4，因此本轮未发送运动指令；未把上次另一底盘的移动记录算作本轮通过。

2026-10-10（虚拟机日期）方式三清障修复：拆分障碍标记与清障观测，清障保留量程内较远及车体高度之外的真实回波；无效深度仍不产生清障射线。Windows 全项目 **129 项测试通过、无跳过**，52 个 Python 文件及 XML/YAML 等结构检查通过。Ubuntu 16.04 / ROS Kinetic / Python 2.7 中 catkin 构建通过；独立 ROS master 的 `clearing_observations.test` 使用真实 VoxelLayer 验证远背景、高处端点、无效深度、前景遮挡、地面射线、同时标记/清障及空标记云替换旧缓存，共 1 个功能测试、7 类场景通过。原 `navigation_smoke.test` 导航和丢失停车也通过，未修改原通过条件。测试没有连接实车速度接口。

现场 `obstacle_return_1791609527` 的深度和占用体素快照已用 `tools/replay_clearing_snapshot.py` 在另一独立 ROS master 重放：相同 4 像素步长下，4.0 m 观测保留 9 个旧体素、4 列；4.5 m 有效观测剩 5 个体素、2 列。两组均观测 40 次地图更新。剩余高度没有有效背景射线，不能改成自由空间；此结果是保存快照的受控重放，不是实机清除耗时或完整动态场景通过。部署重启地图后的空白也不算清障成功证据。

2026-10-09 方式三四项优化：多视角外观模板、强身份匹配下的有界重识别与断裂轨迹处理；相机投影视野约束；里程计实际速度制动包络、VoxelLayer；登记/独立样本评估、录包/隔离回放与人工结论配合的赛道报告。补上导航终态回调锁定和本车持续停稳判定，包括终点附近重新识别后的停车。Windows Python 3.14.7 全项目 **113 项通过，无跳过**；46 个 Python 文件及 launch/test/package XML、配置语义检查通过。真实 Kinetic/Xenial 基础容器中 56 项核心测试：52 项通过，4 项 Python 3 报告加载测试明确跳过（已由 Windows 全量测试覆盖）；方式三节点及工具的 Python 2 语法解析通过。

新增固定基础镜像摘要的 Kinetic Docker 构建和 ROS 导航冒烟测试。**真实 Kinetic catkin 构建及 ROS/cv_bridge/TF/VoxelLayer/DWA 冒烟已在 GitHub Actions 通过**，证据：[15c680f 的三项 CI](https://github.com/aliensco/intelligent_robot_system/actions/runs/38014320329)。测试确认实际生效的 DWA 限速、合成观测下的正向导航和简化底盘移动、丢失后速度归零、影子模式无底盘输出发布者；ROS 报告为 1 个功能测试成功（catkin 汇总另计包装测试，共 2 项、0 失败）。测试通过合成的赛前环视观测建立初始地图覆盖，不等于实机准备或完整赛道绕障。

集成验证中修正了 Kinetic 的 DWA 参数名，并将原地转动最低速度设为第一控制周期可达的值；静态校验也检查此起步条件。本地旧软件源下载遭遇 502/超时后已停止，ROS 验证由上述远端环境完成。未连接 Kinect、底盘或进行真实赛道验收；模板默认仍为空，实机执行需真实模板与完整标定。视野约束仅做短时固定目标投影，VoxelLayer 也不能替代真实盲区检查。

2026-10-09 方式三控制链路开发：在仓库独立 `.venv` 安装 PyYAML 6.0.3、NumPy 2.5.3、OpenCV headless 5.0.0.93，Python 3.14.7 下全项目 **88 项测试通过，无跳过**；其中方式三 56 项。38 个 Python 文件及 XML/YAML/动作结构与方式三配置语义检查通过。覆盖真实 OpenCV 合成图像检测、毫米/米换算、颜色歧义、连续关联、目标丢失与超时、光学坐标变换、停止路径碰撞、影子模式无底盘发布者、未标定拒绝执行、导航失败锁定、最终目标小位移以及整车进入终点。已编写 move_base 滚动 odom 地图与 DWA 配置，但没有运行真实 ROS/navigation，因此不能把核心和消息替身测试称为实际绕障测试。Kinetic 的旧版 OpenCV、Python 2、catkin 构建和整车性能仍待验证；CI 已增加纯核心 Python 2 测试但本次未执行远端 CI。

2026-10-09 方式三开发复查：新增 RGB-D 诊断核心、ROS 回调替身测试、无设备合成数据入口。当前 Windows Python 3.14.7 上共发现 48 项测试，45 项通过，3 项原有视觉测试因缺少 OpenCV/NumPy 跳过；方式三新增 16 项全部通过。静态检查通过 26 个 Python 文件及 launch/package XML、YAML、动作文件。尚未在 ROS 虚拟机运行 catkin_make 或 demo.launch，也未验证 Kinect。下文“32 项全部通过”为此前记录，不能视为本次环境结果。新增 Python 2 核心检查已配置到 CI，本次未实际运行 CI。

在 Windows 开发主机运行 Python 单元测试和项目静态检查；没有连接 TonyPi、Kinect 或 Turtlebot2，也没有 ROS master/导航运行环境。

2026-10-09：Python 3.14 环境下 32 项单元测试通过，含 OpenCV 合成色块/球和无目标图像检查；Python、launch/package XML、YAML、路线与全部五个 d6a 文件结构检查通过。Python 2.7 的核心测试和节点语法检查由 GitHub Actions 单独执行，本机没有 Python 2/ROS Kinetic，不能把 Python 3 通过说成 ROS 集成通过。

检查覆盖：路径长度跟随转角、起步间隔、测量跳变/非有限数拒绝、地图边界/未知区域/占用、定位协方差、扫描有效性、连续观测抓取、动作格式、手臂夹持锁定、夹持信号故障、门控超时/限速/急停、导航失败锁定、目标丢失取消导航与终点停车。节点适配层使用消息替身，不是 ROS 集成测试。

`competition.py --simulate` 已走完整个示例状态序列，动作只打印，没有向真实设备发送命令。生成的两个候选动作仅完成格式验证，不代表机械运动验证。

运行可复查命令：

```bash
python3 tools/validate_project.py
python3 -m unittest discover -s tests -v
python3 humanoid/scripts/competition.py --simulate
```

## 尚待填写的实测值

| 项目 | 测量结果 | 操作者与日期 |
|---|---|---|
| VM ROS 发行版与 Python 版本 | Ubuntu 16.04 / ROS Kinetic / Python 2.7，已连接实机确认 | 2026-10-10 |
| Pi SDK 与 Python 版本 | 待测 | |
| 抓取物体尺寸、重量、颜色 | 待测 | |
| head pan/tilt 零位 | 待测 | |
| LAB 阈值、grab_bottom、grab_area | 待测 | |
| 单次抱物前进距离与偏航 | 待测 | |
| 单次抱物左转、右转角 | 待测 | |
| 抓取→步态→放下的动作衔接 | 待测 | |
| 夹持手臂舵机编号及锁定效果 | 待测 | |
| 右转/放下候选改动与验证 | 待测 | |
| 实测地图 yaml/pgm 与场地照片 | 待补 | |
| 各车 map 坐标一致性 | 待测 | |
| 车体尺寸、通道净宽、队列间隔 | 待测 | |
| 相机标记 ID、边长与可靠识别距离 | 待测 | |
| 单车及整队丢失目标停车时间 | 待测 | |
| 遮挡后原地重新捕获 | 待测 | |
| 前方障碍/无线断链/扫描断链停车 | 待测 | |
| 两次转弯通过、抱持无掉落 | 待测 | |
| 四车终点到达与停稳、跟随方式分类 | 待测 | |

建议每次试跑记录 Git commit、配置文件、地图版本、成功/失败原因，便于另一台电脑复现。失败记录与成功记录都保留。

### Turtlebot target and near-field commissioning fixes (2026-10-10)

- Evaluate identity for all overlapping depth windows before suppressing duplicates. Previously a larger contaminated window could discard the matching window before matching ran.
- Smooth only the twelve achromatic brightness bins within each body quadrant, equally for live and saved templates. Color bins, spatial quadrants, geometry gates, and loss-triggered stopping remain separate. This is tolerance to small exposure shifts, not a general robot classifier.
- For a blocked live-standoff endpoint, `observed_prefix` can choose an intermediate goal whose entire sampled swept footprint is clear. It never jumps across a blocked section, shortcuts the measured leader trail, or treats an intermediate goal as mission completion.
- Optional `commissioning_clearance:=true` installs an operator-inspected-region layer after obstacles and before inflation. `tools/publish_commissioning_clearance.py --confirm-clearance` publishes one fixed odom-frame rectangle from fresh stationary odometry. Default dimensions from the base origin: front 0.60 m, rear 0.50 m, left/right 0.50 m. Inspect the entire specified region first. The layer only fills unknown cells, leaves marked obstacles intact, and expires on either ROS or wall-clock time (default 10 seconds, maximum 15). It is disabled by default and is not a substitute for sensor coverage in continuous competition runs.
- The bounded trial now tolerates float32 serialization error at a speed limit, then clamps the actual output to the exact original limit. Meaningful overspeed still rejects.

Field evidence: 156 offline RGB-D tests pass; ROS Kinetic catkin build succeeds; the real rolling-costmap regression verifies obstacle retention, outside-region unknown space, wrong-frame rejection, expiry back to unknown, and stale replay rejection. Near-view replay detected one target in 24/24 frames; far-view replay detected one target in 6/6 frames, of which one was enrolled and five were held out from enrollment (same static arrangement).

A physical trial ran the real gate output continuously for four seconds without target-loss interruption, but odometry recorded only 0.000384 m and encoder changes [9, 0]; this is NOT successful following or obstacle avoidance. Subsequent live work encountered both camera input timeout and Kobuki serial timeout. Dynamic bypass of a real obstacle and sustained following remain unverified. Current field profile uses 0.8 m standoff, 0.05 m/s linear cap, and visible-base proposal limits (0.13–0.50 m width, 0.075–0.20 m height, 0.17 m support-height ceiling); those are bounds on valid visible depth support, not physical robot dimensions.

Later field result in the same session: after visible-depth-support tuning, float32 bound correction, intermediate-goal handling and low-speed tuning, trial `trial_1791694021.log` actually travelled **0.057103 m**, encoder changes **[677, 662]**, peak measured translation **0.029852 m/s**, peak output **0.05 m/s**, and stopped at the eight-second duration limit. Fresh odometry after the trial reported zero linear/angular velocity and the physical input topic had no publisher. This proves a bounded short approach, not sustained following or successful obstacle bypass. The optional `--resume-after-target-loss` mode emits zero during lost-target/unapproved-gate intervals and resumes only on fresh confirmed target and approved gate; time/distance limits are never reset, prolonged loss and emergency events still stop. Default strict one-shot behavior is unchanged. Latest offline suite: **158 tests passed**. Dedicated ROS master startup restored sensor streams after the camera-owned master exited during restart. An obstacle-placement request is pending for the next physical test.
