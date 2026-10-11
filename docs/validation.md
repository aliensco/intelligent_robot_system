# 验证记录

## 2026-10-11 当前方式三版本与验收边界

当前任务为本车 Kinect RGB-D 自主识别并跟随前方 Turtlebot。最新已经执行的
RGB-D 离线回归为 **295 项全部通过**，`validate_project.py` 完成 **75 个 Python 文件及项目结构校验**；
这不是完整实车绕障或比赛成绩的证明。
下方较早日期与轮次保留历史参数和失败证据，现场运行以本节及
[方式三场地入口](../turtlebot/follow3/README.md) 为准。

最新实现：

- **部分遮挡识别与延迟优化。** 已增加同深度平台/下方空隙辅助、完整底盘模板及
  同帧地面复用，最新帧单槽避免接收队列越处理越旧。当前遮挡失败帧中的同一底盘双框，
  使用共享真实深度支撑和连续底盘表面补充去重；没有补画被遮住的右半车体或删除前景障碍。
  正面开发回归中心最大帧间横跳从 9.34 cm 降至 1.55 cm；转约 30°的新 12 帧均保留真车。
  ROS 只读采样观测年龄 P95 从约 404 ms 降到 230 ms，但场景不同，不能据此声称准确率改善幅度。
  背景边缘误匹配、重遮挡漏检和完整移动绕障仍待验证；详见
  [现场识别诊断与证据](method3_vision_field_diagnosis_20261011.md)。
- **视觉与深度共同识别。** 每帧同时评估 RGB 底盘模板与深度形状；只有空间位置、
  测得深度和真实支撑像素一致且关联唯一时融合，保留不同物体和歧义候选。
  RGB 框改善黑色底盘深度空洞时的中心估计，距离仍使用真实配准深度。
  模板候选的非极大值抑制（NMS）排序加入实测物理宽度与登记宽度的一致性，
  修复 64 像素、约 0.159 m 的半底盘框抑制 140 像素、约 0.352 m 完整框的问题
  （该模板登记宽度为 0.30 m）；原重叠抑制和身份判定阈值没有放宽。
  障碍点云与目标身份识别独立处理。
- **按地图自主规划。** 正常跟随停车点与更远绕行点共同竞争，按全车可达路径长度、
  软障碍代价、间距误差，以及重建路径前 0.25 m 相对前车方向的夹角评分。
  通用栅格连通搜索/Dijkstra 选点，场地模式沿真实重建路径取约 0.30 m 前瞻子目标，
  再交给 Navfn 和 DWA；路径代价权重保持 0.25，没有特定锥桶坐标或预写绕行方向。
  已到达或失败的中间点短期排除；普通规划失败可重新选点。
- **减少反复换目标。** 可用绕行目标最长保持 8 秒；目标明显移动、新障碍挡住终点、
  导航失败或持续视野拒绝仍能解除保持。点云、局部避障与速度检查持续更新。
- **统一运行配置。** `prepare_field_launch` 同步 DWA、门控和转发器速度上限，
  DWA 使用四档动态窗口线速度采样，允许原地转向。较小上限下同时降低最小速度，
  保证第一控制周期存在可达候选。每轮重建跟随状态，保留正常工作的传感器驱动。
  场地 shell 入口通过 `flock --close` 限定单个会话，重复启动会提示已有会话，
  不再因同名 ROS 节点挤掉正在运行的本入口会话；长期保留的可视化/驱动子进程不继承锁。
  入口保留调用参数后再 source ROS 工作空间，修复 `--help` 被 setup 脚本消耗的问题；
  虚拟机上的 `bash -n` 与入口 `--help` 已通过。
- **检查完整 RGB-D 链。** 复用相机前及启动后分别检查 RGB 和配准深度源时间戳；
  深度有数据而 RGB rectifier loader 残留时，重建本相机 namespace 的完整 nodelet 链。
  识别状态记录 RGB/深度/CameraInfo 原始时间戳及处理耗时，旧帧不会因新的状态接收时间变成新帧。
- **无进展时换路。** 目标尚未到达、门控累计限速且里程计没有足够平移/转向进展时，
  取消当前目标、暂时排除该点，等待新地图后重新规划。已到跟随间距的正常停留与有效原地转向
  不判为卡住；这是通用状态处理，没有锥桶坐标判断。
- **自主回正视野。** 前车仍在画面边缘而 DWA 平移弧线会丢失目标时，Gate 可以生成原地转向，
  比较左右转向对图像偏移的改善，并检查整段视野、车体扫过范围与制动包络。
  目标已经丢失时不借此盲目转动。
- **瞬时角速度处理。** 仅 field 模式在实测角速度超过命令上限加 0.03 rad/s 时立即输出零，
  连续新里程计超限 0.10 秒才永久结束会话；超过独立严重异常阈值立即结束。
  恢复要求新里程计持续 0.10 秒回到命令上限内，旧/重复消息不能解锁；暂停不重置行程或时长预算。
  以当前 0.30 rad/s 命令上限，软阈值为 0.33，严重异常结束阈值为 0.40 rad/s；
  后者不是准许运动速度。线速/横向速度、数据失鲜及严格 commissioning 模式的原有停止行为保留。

用户测量当前后车宽约 **0.36 m**、长约 **0.305 m**。本车运行配置采用含余量的
**0.25 m 圆形包络，额外 footprint padding 为 0**；其他车使用各自实测配置。
期望跟随距离 **0.6 m**，正常停车点允许 **0.5 m**，**0.4 m** 停止前进。
这里距离定义为后车底盘原点到前车识别表面点；绕障可以暂时拉开间距。
默认指令上限为 **0.10 m/s、0.30 rad/s**，不代表实际速度一定达到此值。
场地入口允许规划通过未观测栅格，这是明确的调试假设，不能视为已经测得净空；
真实点云障碍与全车轮廓碰撞检查仍参与规划。

较早已结束的试跑 `field_sessions/20261011_174407_35335`：

| 指标 | 实测结果 |
| --- | --- |
| 时长上限 | 45 秒 |
| 累计行程 | 0.13518797 m |
| 累计转角 | 0.426733 rad，约 24.45° |
| 峰值实测线速度 | 0.05544 m/s |
| 峰值实测角速度 | 0.21537 rad/s |
| 结束原因 | `duration_limit` |
| 完整绕障验收 | 尚未通过 |

`20261011_173720` 开头的前次试跑记录中，跟踪状态时间占比约 **98.696%**。
该值描述状态维持时间，不是人工标注的检测准确率、召回率或整场成功率。
45 秒试跑有实际移动和转向，但尚不能证明驶过障碍、持续跟上移动前车、正确到达终点并停稳。
正在进行或没有最终日志的试跑不计为成功。

随后 `20261011_174811_37500` 试跑净位移 **0.798438 m**、累计里程 **0.800125 m**，
记录的 **133/133 个目标有效状态样本为 true**。该轮轨迹接近直线，证明这次布置下有连续的接近运动，
不能据此认定已绕过锥桶，也不能把状态样本有效率当成人工标注的识别准确率。

随后 `20261011_175543` 开头试跑，时长上限 **60 秒**：

| 指标 | 实测结果 |
| --- | --- |
| 累计行程 | 0.36125477 m |
| 累计转角 | 0.69516464 rad，约 39.83° |
| 峰值实测线速度 | 0.07676288 m/s |
| 峰值实测角速度 | 0.31922072 rad/s |
| 角速度超限暂停次数 | `angular_overspeed_pauses: 0` |
| 结束原因 | `duration_limit` |

该轮后半程因 `ambiguous` / `reconfirm` 反复暂停，尚未形成持续完整绕障的通过证据。
角速度峰值没有越过 0.33 rad/s，因此不能把本轮记为角速度瞬时暂停/恢复的实车验收；
新增无进展换路和 Gate 自回正同样需要对应状态、轨迹和完整场景证据，未证实触发的功能不记为通过。

更新识别和路径前瞻后，`20261011_180259` 开头试跑记录如下：

| 指标 | 实测结果 |
| --- | --- |
| 累计行程（`motion.log`） | 0.6692445 m |
| 净位移 | 约 0.6137 m |
| 相对起始方向的右侧位移 | 约 0.203 m |
| 跟随距离变化 | 约 1.437 → 0.866 m |
| 目标有效状态占比 | 96.18%；此前对比轮次为 38.01% |
| 最长目标丢失时间 | 0.603 秒；此前对比轮次为 13.61 秒 |
| 地图推算的最小间隙 | 约 0.019 m，非实物测距 |
| 角速度超限暂停 | 1 次，随后恢复 |

同轮 bag 按采样点累计约 0.672 m，与 `motion.log` 存在采样差异，正式行程记录以上表日志为准。
有效状态占比描述程序输出，前后轮次对比不是人工标注的识别准确率。
地图间隙也不等于实测车体与障碍距离，不能凭这个值证明擦边无碰撞。
该轮已观察到侧向绕行趋势、接近前车及角速度零输出暂停后的恢复，
但没有完整越过障碍并持续跟随至终点的证据，仍不记为完整绕障通过。

最新 `20261011_180505` 开头的收尾试跑，时长 **20 秒**，累计 **0.220309 m**，
峰值实测线/角速度 **0.0724983 m/s / 0.379958 rad/s**，角速度软超限暂停 **1 次**后恢复，
以 `duration_limit` 结束。角速度峰值是实际响应记录，不是提高后的命令上限。
收尾静态画面为 `tracking`，显示一个完整 **224 × 68 像素**目标框，目标深度约 **0.708 m**；
该深度不直接当作底盘坐标系的跟随距离。结束后机器人停稳，动态场景仍待现场测试。

下一步按 [可重复验收说明](method3_acceptance.md) 记录直行、左绕、右绕、多障碍、
遮挡和动态穿行，各场景分别输出 `PASS` / `FAIL` / `INCOMPLETE`。
预算到期、厘米级移动、缺少新鲜 RGB/深度或障碍间隙、未到达预定终点、未实际测试的场景
均不能直接通过；全队终点仍需额外现场验收。

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
# 方式三场地绕障修复（2026-10-11）

对 `20261011_165718_11671/telemetry.bag` 中地图回放：原先 0.5–0.6 m
候选停车环被障碍覆盖，直线前缀目标导致导航失败，底盘仅移动 0.0165 m。
新增可达中间点选择（轮廓膨胀、连通区域检查），保留已占用和未知区域的输入语义；
允许暂时增大跟随间距绕障。场地配置修正最低线速度、转向和停车容差，并在普通导航
失败后等待更新地图重试，忽略已经取消的旧目标回调。

180 项 RGB-D 离线测试通过，包括实际失败地图回归、不可达孤立终点、阻塞停车环、未知区域、失败目标避让、
新地图重试、急停和目标短暂超时恢复。部署后的第一轮实际移动 0.2146 m、累计转角
0.5309 rad，因短暂目标消息超时结束；该超时已改为场地模式下零速度等待恢复。
这证明运动和规划取得进展，不等同于完整绕过锥桶或完整赛道验收。

第二轮 `20261011_170640_15492` 验证了 `navigation_retry_wait` 后自动换点恢复，
但最大前进指令仅 0.0182 m/s，两轮编码器无增量。场地 DWA 改为两档线速度采样
（动态窗口端点，静止时为 0 和 0.05 m/s），保留原地转向，防止持续选择无法起步的小速度。
第三轮 `20261011_170818_16228` 实际移动 0.0060 m 后以 `stale_sensors` 结束；
同时相机断流，内核有 xHCI `Transfer event TRB DMA ptr not part of current TD` 和
`unknown event type 37`。这一轮不构成完整绕障成功证据，USB 数据链路仍阻碍实车验收。
