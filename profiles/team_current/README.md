# 当前方式三共享配置

这三个文件与 2026-10-11 已验证部署包的运行配置一致，供 GitHub 克隆后直接安装使用。

- `sensors.launch`：Kobuki 驱动、实际速度输入重映射和 RGB-D 探针。
- `turtlebot_follow_current.launch`：相机安装 TF、配准深度话题和跟随启动参数。`settings` 使用同目录相对路径。
- `turtlebot_follow_current.yaml`：当前 Turtlebot 的 4 组身份参考、2 组 RGB 模板，以及深度、关联、跟随与规划设置；模板已内嵌，不依赖额外图片下载。

`bash install.sh` 在首次安装时将它们复制到根目录 `runtime/`。再次安装保留本机配置；Git 更新这些共享文件后，也不会覆盖已有 `runtime/`。用 `diff -u profiles/team_current/turtlebot_follow_current.yaml runtime/turtlebot_follow_current.yaml` 查看差异，再合并本机需要的参数。

相机 TF 对应这次实测安装，不是所有同型号底盘通用的外参。三个 `*_calibrated` 标志保持 false，完整场地验收尚未完成。修改后希望队友收到的参数，应同步到此目录提交；本机录包、日志、临时试跑配置继续留在忽略的 `runtime/` 中。
