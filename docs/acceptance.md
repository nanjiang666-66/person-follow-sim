# 第一版验收记录

## 环境与版本

先记录 Ubuntu、ROS、Gazebo 版本，以及 `records/installed_versions.txt`。保留每轮运行场景、启动参数、CSV、必要的 rosbag 和 MP4。

所有命令在 `ROS_DOMAIN_ID=17` 下执行。验收空场景使用独立仿真；没有真实底盘连接。

## 通过条件

| 项目 | 方法 | 第一版通过条件 |
|---|---|---|
| 编译与依赖 | `scripts/build.sh` | 4 个本地包编译成功；C++ 测试和 CSV 检查器测试通过 |
| Action 服务器 | lifecycle/action list | following_server active；FollowObject 类型正确 |
| 速度发布者 | `ros2 topic info /cmd_vel --verbose` | 只有 follow_supervisor 发布最终速度 |
| 距离保持 | stationary/away | 末段 8 秒中位绝对误差 ≤0.15 m；无持续振荡 |
| 靠近与恢复 | approach | 人近时不前进、不倒车；人远离后继续 |
| 转向 | lateral/turn | 方向正确，目标留在视野内；大偏角先转向 |
| 目标与深度失效 | occlusion/invalid_depth | 失效期间指令归零；不把旧坐标当新测量 |
| 短时恢复 | 1/2/3 秒失效 | 收到新测量且连续确认后恢复；不靠真值跟随 |
| 数据停发 | pause_publishing | 最多约0.5 s +一个控制周期后归零 |
| 仿真暂停 | Gazebo Pause | 墙上时间超时仍归零；恢复后重新确认 |
| 软件急停 | emergency_stop | 指令归零；解除后重新确认 |
| 实际停车 | 观察 /odom、车轮 | 零指令后合理减速至静止，记录实际延迟 |
| 正常结束 | Ctrl+C | 监督节点结束前发送零指令，小车停止 |

如果虚拟机严重卡顿，先保留原始失败记录，再关闭 Gazebo GUI 重试。不能仅放宽输入超时来掩盖卡顿；应先检查实时因子、CPU 和话题更新率。

## 查看数值的命令

```bash
source /opt/ros/humble/setup.bash
source ~/person_follow_ws/install/setup.bash
export ROS_DOMAIN_ID=17
ros2 topic echo /follow/target_base
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic hz /person/target_point
```

`/follow/target_base` 中 x 向前、y 向左。相机光学坐标中 x 向右、y 向下、z 向前。画面右侧的人对应负的车身角速度；图像左侧对应正角速度。

## 验收记录

2026-10-04首批7份CSV及新增away均已使用原检查器复核，normal_exit又按独立退出规则检查通过。结果见[原始记录复核](recording-review-20261004.md)和[正常退出结果](normal-exit-test.md)。6个场景完整记录PASS，正常退出已补齐；旧away暂停及approach长运行停顿证据保留，后者按用户要求复现后再查。第一版作为[带已知问题的基线](baseline-20261004.md)保存。

| 场景 | CSV/bag/视频 | 距离误差 | 停车延迟 | 恢复情况 | 结论/待修复 |
|---|---|---|---|---|---|
| stationary | `stationary_20261003_221931.csv`；1367.01仿真秒、27388行 | 末段8秒中位绝对误差0.029 m | 未单独量化 | 到达并输出零线/角速度 | 完整CSV PASS；最大指令0.200 m/s、0.012 rad/s |
| away | `away_20261004_141800.csv`；99.00仿真秒、2271行；旧暂停记录另保留 | 末段8秒中位绝对误差0.024 m | 未单独量化 | 持续远离后收敛，末段命令为零，新鲜odom近静止 | 新完整CSV PASS；最大命令0.200 m/s、0.000 rad/s；末段距离中位数1.524484 m，最大绝对实际线速度0.000384 m/s |
| approach | `approach_20261003_224244.csv`；完整2026.02仿真秒、40770行；此前1079.2秒/21626行检查PASS | 不以末段误差判定；诊断值约0.040 m | 未单独量化 | 近距不前进、不倒车及远离后恢复检查未报错；用户反馈停车平稳 | 完整CSV FAIL：后续5行命令年龄超限且均为零，最长采样间断约15.88秒；用户要求暂留、复现再查 |
| lateral | `lateral_20261004_114134.csv`；完整645.34仿真秒、14900行；此前400秒/9032行检查PASS | 不以末段误差判定；诊断值约0.086 m | 未单独量化 | 6557/6557个有效转向样本命令朝向目标；模拟点水平偏角最大4.40° | 完整CSV PASS；最大指令0.200 m/s、0.136 rad/s；数值分析不等于全人体真实视野验收 |
| turn | `turn_20261004_124943.csv`；43.62仿真秒、1000行 | 末段8秒中位绝对误差0.017 m | 未单独量化 | 434/434个有效转向样本命令朝向目标；模拟点水平偏角最大2.04° | 完整CSV PASS；最大指令0.200 m/s、0.077 rad/s；全程实际响应与抖动未量化 |
| occlusion | `occlusion_20261004_115403.csv`；150.07仿真秒、3447行 | 末段8秒中位绝对误差0.028 m | 零命令到近静止约0.047–0.140 s，50ms采样估计 | 3个失效区间及恢复通过 | 完整CSV PASS；最大指令0.200 m/s、0.000 rad/s |
| invalid_depth | `invalid_depth_20261004_121924.csv`；完整158.66仿真秒、3651行；此前119.7秒/2750行检查也曾PASS | 末段8秒中位绝对误差0.021 m | 零命令到近静止约0.092–0.093 s，50ms采样估计 | 完整文件3个失效区间及恢复通过 | 完整CSV PASS；最大指令0.200 m/s、0.000 rad/s；此前两次终端检查不计作两次独立录制 |
| 手动停发 | 用户终端pause_publishing参数修改、状态与Twist，2026-10-04 | 未统计 | 等待3秒后查询为零指令，精确触发/实际停车延迟未测量 | 解除停发成功；用户反馈“一切正常” | 基础停发停车与恢复检查通过；状态target_not_reliable，与visible消息也过期的实现一致 |
| 仿真暂停 | 用户回传/follow/state、Twist与恢复动作，2026-10-04 | 未统计 | 暂停后查询为零指令；约0.5秒触发边界未量化 | 用户确认恢复后小车继续跟踪 | 基础仿真时钟暂停与恢复检查通过；状态clock_unavailable_or_paused，Twist各分量均为0 |
| 软件急停 | 用户终端emergency_stop参数、状态与Twist及动作反馈，2026-10-04 | 未统计 | 等待1秒后查询为零指令；精确指令/实际停车延迟未量化 | 用户确认：目标仍较远，解除后小车恢复跟随 | 基础急停零指令与恢复检查通过；/follow/state=emergency_stop，Twist各分量均为0 |
| 正常结束 | `normal_exit_20261004_143039.csv`；535行、23.31仿真秒；脚本与启动日志PID40248一致 | 不适用 | 首次记录零指令到持续近静止约0.251现实秒，50ms采样估计 | 正常退出后继续记录约17.455现实秒；目标远离、小车保持停止 | 独立退出检查PASS；退出前命令约0.100 m/s，最后命令为零，新鲜odom保持近静止；完整结果见normal-exit-test.md |

用户verbose输出已确认/cmd_vel只有follow_supervisor一个发布者，follow_diff_drive和follow_monitor订阅。补充转向分析选取|cmd_w|>0.03 rad/s且|bearing|>1°的样本；相机水平半视野为配置值34.38°。近静止估算仅使用年龄<0.1秒的odom，阈值为|v|≤0.01 m/s、|w|≤0.02 rad/s并保持约0.10–0.15秒；不是实车制动时间或精确失效触发延迟。

CSV 的 `point_age_s`、`visible_age_s`、`odom_age_s` 是接收后的墙上时间，控制节点还会另行检查消息自身的 ROS 时间戳。CSV 检查器允许 0.12 秒的不同话题订阅到达误差，但控制节点不会主动等待这段时间才停车。

`occlusion` 与 `invalid_depth` 在模拟的检测状态上不同，但失效时均令 `/person/visible=false` 并暂停新XYZ。当前控制节点不订阅 `/person/detected`，两项采用相同的停车与恢复分支、相同的CSV判定规则。CSV不记录detected，上述输出不能单独证明该话题的检测状态；也不代表真实深度算法通过了新一轮遮挡测试。数据完全停发时可能保留最后的visible=true，因此需要另行验证数据过期停车。

状态判定说明：监督节点先将visible标志与其消息接收超时合并，再进入控制门限判断。整个/person流停止时，即使最后的visible值为true，超时后合成的input.visible也为false；其检查早于target_fresh，所以会显示target_not_reliable。target_stale可用于可见标志持续新鲜、XYZ过期的情况。上述手动查询确认了停发期间零指令，尚未测得约0.5秒的超时边界。

## 适用范围

该验收检查正常运行、输入失效和正常结束。不能证明任何进程崩溃时底盘都会停止：强制杀死监督节点可能使 Gazebo 差速插件保留上一条指令。实车阶段必须由底盘驱动/固件独立执行指令超时停车，不能只依赖上层节点。
