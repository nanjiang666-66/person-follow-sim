# 来源与版本

- 本项目控制监督节点、模拟目标、记录脚本和启动配置为当前任务新建。
- 底盘几何参考用户共享目录中的独立 `robot_nav_course` 项目；本项目没有修改该项目，也没有复用其 A*。
- 运行时复用 `open-navigation/opennav_docking` 的 Humble `opennav_following`，Apache-2.0；源码后备版本记录于 `UPSTREAM_LOCK.json`。
- `third_party/opennav_docking` 是被 Git 忽略的本地源码检查副本，不随本仓库提交；其原始版权和许可证保留在副本中。
- Gazebo、ROS 2、Nav2 等外部依赖继续使用各自的许可证。

当前任务仅创建本地仓库；没有创建远程仓库或推送文件。
