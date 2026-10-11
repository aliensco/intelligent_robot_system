# 方式三部署包快速启动

用途：后车用自己的 Kinect RGB-D 识别并跟随前方 Turtlebot，通过实时点云和规划器避障。前后车不需要通信网络。

要求：**已经装好 Ubuntu 16.04 + ROS Kinetic 的虚拟机**。包是源码、当前配置和安装/启动入口，不是操作系统镜像；当前版本仍需场地验收，尚不能声明比赛全流程通过。

## 1. 从 GitHub 获取并安装

团队协作可直接克隆已包含当前配置的交接分支，不需要额外下载模板包：

```bash
git clone --branch codex/method3-team-handoff --single-branch \
  https://github.com/aliensco/intelligent_robot_system.git ~/method3-team-20261011
```

也可以解压之前的部署包。两种方式都进入 `method3-team-20261011` 后执行：

```bash
cd ~/method3-team-20261011
bash install.sh
```

安装器会安装缺失的 apt 依赖，配置通用设备规则和用户组，在包内建立 `workspace/` 并构建。按提示输入本机 Ubuntu 密码；若提示用户组刚添加，注销后重新登录。已有系统依赖和设备权限，只需重建时使用：

```bash
bash install.sh --skip-system
```

重装保留 `runtime/` 中的现场配置修改。

## 2. 将 USB 连接到 Ubuntu

VMware → **VM → Removable Devices**，连接：

- `Future Devices iClebo Kobuki`：后车底盘。
- `Microsoft Xbox NUI Camera`：后车 Kinect，相机保持供电。

每台 VM 只连接自己的一套底盘和相机。不用配置前车 IP，也不用共享 ROS master。

## 3. 打开识别画面和点云

在 Ubuntu 图形桌面的终端执行：

```bash
bash run.sh --view-only
```

入口自动加载工作区并启动底盘/相机、识别画面和 RViz 点云；此模式不启动运动转发。初始化结束返回终端后，窗口可继续保留。

## 4. 开始跟随避障

将前车放在 Kinect 能看到的位置，然后执行：

```bash
bash run.sh
```

不需要再分别启动底盘、相机、旧试跑脚本，也不需要另行 `source`。**在本终端按 Ctrl+C 结束运动**。相机和可视化默认保留，下一轮继续运行同一条命令即可。

默认单轮最多 300 秒或累计 5 米，线速度上限 0.10 m/s、角速度上限 0.30 rad/s。当前目标期望距离 0.60 米，正常停车候选可到 0.50 米，过近停止阈值 0.40 米。这是定位原点到目标参考点的距离，不是保险杠净空。

短轮测试示例：

```bash
bash run.sh --duration 60 --distance 2 --max-speed 0.08 --max-turn 0.30
```

完整选项：

```bash
bash run.sh --help
```

目标丢失、路径受阻或输入过期时可能停车；当前允许规划经过未观测栅格，同时保留真实障碍检查。全程动态绕障和遮挡恢复仍需实测，不能仅凭点云窗口出现判断已经通过。

## 5. 调参、反馈与开发

- **改配置**：`runtime/turtlebot_follow_current.yaml`；相机安装 TF 在同名 `.launch`。交付原件在 `profiles/team_current/`。修改后重新运行入口。
- **找日志**：终端打印的 `SESSION_LOG` 目录，位于 `runtime/field_sessions/`。优先看 `status.jsonl`、`motion.log`；驱动问题看相应启动轮次的 `camera.log`、`base.log`。
- **没运动**：查看结束原因和跟随状态。可能已经到距、未识别到目标、驱动无数据或规划受阻；把整轮日志和实际启动命令交给同学。
- **没窗口**：在 Ubuntu 图形桌面运行，查看 `preview.log` / `pointcloud.log`。
- **保留配置证据**：每轮 `field_settings.yaml`、`field.launch` 是最终生效参数。默认遥测录包不含原始 RGB/深度；识别复现需另采同步图像。

包内有源码与 Git 历史，可直接创建自己的分支：

```bash
git status
git remote -v
git checkout -b team/your-name
```

按实际仓库权限推送分支或 Fork，使用自己的账号，不交换凭据。最新原始采集数据不随包，已有回归样本在 `tests/fixtures/`。

更多配置、故障处理和协作说明见 [团队交接](method3-team-handoff.md)，场景判定见 [验收说明](method3_acceptance.md)。
