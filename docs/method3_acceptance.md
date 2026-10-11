# 方式三自主跟随避障验收

`tools/evaluate_rgbd_acceptance.py` 将现场 JSONL 或 ROS bag 转成逐场景
`PASS` / `FAIL` / `INCOMPLETE` 报告。它不控制小车，复用既有
`evaluate_rgbd_trials.py` 的百分位计算；旧录包/回放工具继续可用。

默认要求六类场景各最后连续三次通过：

| 场景标签 | 现场内容 |
| --- | --- |
| `straight_follow` | 前车连续移动，后车跟随后停稳 |
| `obstacle_left` | 左侧通道可通行，实际从左边绕过障碍 |
| `obstacle_right` | 改变布置，实际从右边绕过障碍 |
| `multiple_obstacles` | 至少两个障碍，改变形状、间距和顺序后自主规划 |
| `occlusion` | 短暂遮挡、停车、恢复识别后继续 |
| `crossing` | 其他车辆或障碍横穿，重新规划后继续 |

每次录包前明确终点坐标（同一次启动的 `odom`），到达并停稳后结束。
记录真实遇到的场景和人工观察：身份正确、无碰撞、无人推车干预。
不得把摆了障碍但没有驶过的试验标为已完成绕障。
报告不换算比赛分数；全队终点及比赛规则仍沿用原有
`evaluate_rgbd_trials.py` 的 `team_finish` 验收。

## 使用

把 manifest 与录包一起保存。下面是一项试跑记录；尚未录制的场景会保持
`INCOMPLETE`，不会自动认为通过。

```json
{
  "repeats": 3,
  "trials": [{
    "scenario": "multiple_obstacles",
    "log": "multiple-01.bag",
    "terminal_reason": "operator_completed",
    "endpoint": {"xy": [2.0, 0.5], "tolerance_m": 0.15},
    "footprint_radius_m": 0.26,
    "observed_obstacle_count": 3,
    "criteria": {"min_travel_m": 1.5, "max_target_loss_s": 3.0},
    "verdict": {
      "scenario_exercised": true,
      "collision_free": true,
      "correct_identity": true,
      "no_intervention": true,
      "route_completed": true
    }
  }]
}
```

`operator_completed` 仅在实际完成预定路线并人工结束时填写。若使用当前限时
场地转发器，同时填写 `"motion_log": "motion.log"`；工具优先读日志中真实的
`final_reason`。`duration_limit`、`travel_budget`、预检超时不能表示整段验收完成。
路线未到终点或没有测到连续半秒停稳也不能通过。

```bash
# Ubuntu/ROS bag（先 source 工作空间）；JSONL 不依赖 ROS，Python 2/3 均可。
python tools/evaluate_rgbd_acceptance.py /data/manifest.json --output /data/acceptance.json
```

退出码为：0 通过，1 有明确失败证据，2 证据不足或未执行。
报告同时保留 `failures` 与 `missing`。默认检查有效时长至少 5 秒、实际里程至少
0.5 米；这只是拒绝厘米级试动，正式赛道应在 `criteria.min_travel_m` 设置真实需求。
默认期望间距 0.6 米、最低 0.4 米，误差 P95 上限 0.6 米；绕障场景可根据实测
合理填写 `gap_error_p95_limit_m`，勿通过扩大阈值掩盖追不上前车。

## 证据要求

bag 读取以下默认话题，可通过 manifest 的 `topics` 字典覆盖同名键：

| 键 | 默认话题 |
| --- | --- |
| `odom` | `/odom` |
| `sensor` | `/mobile_base/sensors/core` |
| `rgb` | `/camera/rgb/image_rect_color` |
| `depth` | `/camera/depth_registered/hw_registered/image_rect` |
| `grid` | `/method3_motion/move_base/local_costmap/costmap` |
| `target` | `/method3_motion/target_pose` |
| `target_valid` | `/method3_motion/target_valid` |
| `perception` | `/method3_motion/perception_status` |

RGB 和深度需有原始消息时间戳，默认至少 95% 的运行时间保持 0.5 秒内的新鲜度。
可录制原图，或使用新版 `perception_status` 的 `rgb_stamp`、`depth_stamp`
轻量证据。后者由识别节点在检查同步 RGB-D 输入后记录，不能用状态消息接收时间替代。
旧场地 telemetry.bag 若未录制图像消息且没有这些字段，会标 `insufficient_fresh_rgb/depth`；
RViz 留着点云、节点存在、`tracking` 字符串都不能代替输入在持续更新的证据。
读取 bag 不播放速度、不会连接底盘。

目标、地图和里程计必须属于相同坐标系。其他坐标系应先用录包中 TF 正确转换；
本工具不猜测缺失变换。地图间隙为机器人圆形足迹到已知致命障碍栅格的保守距离，
已扣除半个格子对角线，不是物理无碰撞的证明。未知格不被当作测得的障碍间隙。
没有障碍距离证据的绕障场景保持 `INCOMPLETE`。

纯 JSONL 可使用同等证据字段，一行一个样本：

```json
{"kind":"sample","t":100.0,"odom_stamp":100.0,"rgb_stamp":100.0,"depth_stamp":100.0,"grid_stamp":100.0,"sensor_stamp":100.0,"target_stamp":100.0,"target_valid_stamp":100.0,"target_valid":true,"xy":[0.0,0.0],"speed":0.1,"angular_speed":0.0,"target_distance_m":0.6,"costmap_clearance_m":0.05,"bumper":0,"cliff":0,"wheel_drop":0}
```

必须填入真实采样时间和实测数值，不得用当前时间替代旧消息时间戳。
该样例仅说明格式，不是现场证据。原 recorder 的 JSONL 缺少部分源时间戳，
仍可由旧工具评估；本工具会明确报告缺项，不补造新鲜度。
