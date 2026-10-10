# 人体跟随：Humble + Gazebo Classic 11 第一版

独立项目，配合已有 `d435i_person_tracking` 的 ROS 2 接口。控制节点、模拟目标和记录节点使用 **C++17**；Python 仅用于 ROS 启动文件和离线 CSV 检查。

**当前状态：6个场景完整CSV与正常退出检查已通过，第一版保存为[带已知问题的仿真基线](docs/baseline-20261004.md)；approach长运行停顿保留，进入真实视觉接口联调。** 详细结果见[原始记录复核](docs/recording-review-20261004.md)和[实施状态](docs/implementation-status.md)。

实车到位前使用[真实视觉接口观察入口](docs/vision-preview.md)：固定D435i、测试TF/零odom、最终速度发布到/follow_preview/cmd_vel。2026-10-06基础话题和TF已由虚拟机输出确认连通，真人动态观察与连续CSV检查待完成。

## 1. 第一版能验证什么

- 两轮差速底盘、固定相机；通过车身转向保持人位于相机视野内。
- 默认跟随水平距离 **1.5 米**。基准是 `base_footprint` 到人体测量点的地面投影，不是镜头到人体的三维直线距离。
- 接近 1.52 米时停止前进；人再次远离至 1.62 米后允许继续跟随。存在小幅滞回，避免反复启停。
- 人偏向一侧时转向；偏角较大时先原地转向。人在近处时可以转向，但不前进、不倒车。
- 目标丢失、没有可靠深度、数据过期、TF 不可用时，输出零速度。
- 恢复后至少收到 3 个不同时间戳的新测量，且持续约 0.2 秒，才重新允许控制。
- 空场景先验证跟随闭环。障碍物绕行、真实相机图像仿真和实车接入在后续阶段。

这里的“人”是 RViz 中的圆柱标记和模拟 `/person/*` 测量，**不是 Gazebo 中运行 YOLO 的人体模型**。模拟测量根据小车实际 `/odom` 计算，并受相机视野限制。`/sim/target_ground_truth` 仅供画图和评估，控制节点不订阅它。

## 2. 结构和复用的包

```text
模拟目标 / 后续真实 person_vision
   /person/target_point + /person/visible
                    │
                    ▼
follow_supervisor（C++）──TF──► odom 中的目标 PoseStamped
                    │                 │
                    │                 ▼
                    │       官方 opennav_following
                    │                 │
                    │       /follow/cmd_vel_raw
                    ▼                 ▼
           状态检查、转向和速度限制
                    │
                 /cmd_vel
                    │
          Gazebo 官方差速驱动插件
                    │
              /odom + TF 反馈
```

| ROS 包 | 作用 |
|---|---|
| `person_follow_control` | 相机点转换、跟随 Action 客户端、最终速度控制 |
| `person_follow_sim` | 受视野限制的目标模拟、CSV 记录 |
| `person_follow_description` | 差速车 URDF、空世界、RViz 配置 |
| `person_follow_bringup` | 仿真及控制启动文件、参数 |
| `opennav_following` / `opennav_following_msgs` | 官方 Humble 跟随服务器及 Action 接口 |
| `gazebo_ros` / `robot_state_publisher` / `tf2` | 物理仿真、里程计和坐标变换 |

Humble 的跟随包来自 [open-navigation/opennav_docking 的 humble 分支](https://github.com/open-navigation/opennav_docking/tree/humble/opennav_following)。安装脚本优先用 apt 发布包；若当前源没有该包，编译锁定的官方 Humble 源码。下载时优先从共享目录中已核验的 `third_party/opennav_docking` 获取，避免虚拟机再次访问 GitHub；没有本地副本时才联网下载。版本记录见 `UPSTREAM_LOCK.json` 和 Ubuntu 的 `~/person_follow_ws/records/installed_versions.txt`。没有移植 Jazzy 源码，也没有自行实现 A*。

## 3. 先确认共享目录

Windows 源码路径：`<Windows共享目录>\person_follow_sim`。

Ubuntu 正常对应：`/mnt/hgfs/ubuntu-workspace/person_follow_sim`。

```bash
vmware-hgfsclient
findmnt -T /mnt/hgfs/ubuntu-workspace
ls -ld /mnt/hgfs /mnt/hgfs/ubuntu-workspace
ls /mnt/hgfs/ubuntu-workspace
ls /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts
```

只有 `vmware-hgfsclient` 列出共享名称，不能证明共享目录已挂载。若名称存在、`/mnt/hgfs` 为空且没有对应挂载，可手动挂载：

```bash
sudo mkdir -p /mnt/hgfs
sudo vmhgfs-fuse .host:/ /mnt/hgfs -o subtype=vmhgfs-fuse,allow_other
ls /mnt/hgfs/ubuntu-workspace
```

确认目录可读后再安装依赖。不要从旧项目目录复制 `.git` 或使用 `sudo cp` 覆盖源码。

## 4. 首次安装和编译

在 Ubuntu 普通终端执行，不需要激活之前 YOLO 的 Python 虚拟环境。

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/check_environment.sh
```

确认输出后安装依赖。这一步使用 `sudo apt`，可能需要输入 Ubuntu 密码，并需要网络访问 apt 或 GitHub。

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/install_dependencies.sh
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/build.sh
```

源码保留在共享目录，编译、安装、日志和录制文件写入 Ubuntu 自己的 `~/person_follow_ws/`。编译并发已限制，适合 VMware CPU 环境。依赖脚本不会配置 GitHub 上传。

如果此前 TF 包停留在旧版本，单独安装新 `tf2_geometry_msgs` 可能出现 `tf2_ros/buffer_interface.hpp` 找不到的错误。脚本会显式更新相关 TF 包、检查头文件，并重新生成上游 CMake 缓存；不要手动把系统中的 `.h` 改名或复制成 `.hpp`。

## 5. 第一次运行：静止目标

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/run.sh stationary
```

应该打开 Gazebo 和 RViz。Gazebo 显示差速小车；RViz 显示小车、目标和蓝色相机视野线。目标起始在小车前方约 2.5 米，小车逐渐靠近并在约 1.5 米停车。

脚本默认 `ROS_DOMAIN_ID=17`。所有用于检查本项目的其他终端也要设置为 17。这样可以避开其他默认域的课程小车或真实视觉节点。

如果 RViz 只显示小车，展开 `Target and camera view`，确认 `Topic` 为 `/sim/markers`。Humble 的 MarkerArray 使用 `Topic` 字段；该字段配置错误时，显示项可能看似正常，却订阅了默认的其他话题。正常应持续收到目标圆柱、文字和蓝色视野线。Gazebo 此阶段只包含小车的物理模型。

如果鼠标无法旋转/平移视角且顶部没有 `Move Camera`，是配置缺少 `Tools` 列表。本配置已补齐视角、选择、聚焦工具和 `Views` 面板。选中顶部 `Move Camera` 后，在三维画面中左键拖动旋转、按住 Shift 加左键拖动平移、滚轮缩放；`Views` 面板也可以直接调整 Distance、Yaw、Pitch。

如果启动终端报 `Do not call canTransform or lookupTransform with a timeout unless you are using another thread for populating data`，请重新运行 `scripts/build.sh` 后重启仿真。监督节点已改为启用 TF 监听器自带的独立线程，使用它的专用回调组接收 `/tf` 和 `/tf_static`。坐标变换仍为零等待，保留测量的原时间戳；如果还有 `Target TF unavailable:` 警告，需要根据完整警告继续检查缺失的坐标系或时间差。

如果状态每约0.1秒在 `confirming_target` 和 `odom_unavailable` 之间切换，检查 `ros2 param get /gazebo publish_rate`。Gazebo 默认10 Hz的 `/clock` 可能比50 Hz里程计时间戳落后近0.1秒，超过本项目0.05秒的未来时间容差，从而反复清空恢复确认。本仿真已通过 `config/gazebo.yaml` 在启动时将时钟发布频率设为100 Hz（按仿真时间计）；实际墙上频率还受仿真运行速度影响。该参数需要重启 Gazebo 才生效。监督节点现在也会打印 `Odometry rejected`，区分里程计样本无效、采集时间戳差异和接收超时。

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source ~/person_follow_ws/install/setup.bash
export ROS_DOMAIN_ID=17

ros2 node list
ros2 lifecycle get /following_server
ros2 action list -t
ros2 topic info /cmd_vel --verbose
ros2 topic echo /follow/state
```

- `/following_server` 应是 `active`。
- `/follow_object` 的类型应是 `opennav_following_msgs/action/FollowObject`。
- `/cmd_vel` 应只有 **`follow_supervisor` 一个发布者**。
- 常见状态为 `confirming_target`、`waiting_follow_server`、`following`；进入监督节点的距离保持范围时为 `hold_distance`。官方服务器也会在自身0.03米到达容差内提前给出零速度，此时监督状态可能仍是 `following`。判断是否停稳应同时看实际距离、最终 `/cmd_vel` 和 `/odom`，不能只要求状态名必须变为 `hold_distance`。

按 `Ctrl+C` 结束这一轮，重新启动下一种场景。不要同时开两套仿真。

VMware 图形较慢时可以保留 RViz、关闭 Gazebo 界面：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/run.sh stationary gui:=false
```

## 6. 验收顺序

每轮启动时终端打印 CSV 路径。时长按 **仿真秒** 计算；VMware 运行较慢时，实际等待时间会更长。达到时长后按 `Ctrl+C`，检查对应记录。

| 顺序 | 场景参数 | 至少录制 | 重点观察 |
|---|---|---:|---|
| 1 | `stationary` | 30 仿真秒 | 从 2.5 米靠近并在目标距离附近停车 |
| 2 | `away` | 50 仿真秒 | 人远离时连续跟随，停止后距离收敛 |
| 3 | `approach` | 60 仿真秒 | 人靠近时停车、不倒车；再次远离后继续 |
| 4 | `lateral` | 45 仿真秒 | 左右转向方向正确，保持目标在视野内 |
| 5 | `turn` | 40 仿真秒 | 人沿弧线运动，小车转向并跟随 |
| 6 | `occlusion` | 45 仿真秒 | 1、2、3 秒丢失期间停车，恢复后继续 |
| 7 | `invalid_depth` | 45 仿真秒 | 检测到人但深度无效时停车，恢复后继续 |

`approach` 的时序：0–10秒目标站定，10–16秒以0.1米/秒靠近0.6米，16–30秒停留，30–55秒以0.1米/秒远离，之后站定。靠近幅度使目标在小车首次停车后仍处于模拟相机的有效测量范围内。所有时刻均从模拟目标节点开始收到仿真数据算起。

例如：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/run.sh approach
# 结束后把下面文件名替换为本轮打印的实际路径
python3 /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/check_recording.py \
  ~/person_follow_ws/records/approach_YYYYMMDD_HHMMSS.csv --scenario approach
```

CSV 检查包括：速度上限、不倒车、输入失效后零指令、距离收敛、场景中实际发生运动，以及遮挡后的恢复。它不会把“一直不动”判为跟随成功。默认距离误差验收值为末段 8 仿真秒的中位绝对误差不超过 **0.15 米**；这是一版的验收目标，还需用实际 Gazebo 记录确认。

同时观察 `/odom` 中的实际速度。零指令发布之后，车轮仍可能有短暂减速过程；CSV 的命令检查不能替代对实际停车行为的检查。验收表见 [完整测试说明](docs/acceptance.md)。

## 7. 手动故障测试

先保持 `away` 场景运行。在另一终端完成第 5 节的环境设置后，每次只做一个测试。

```bash
# 软件急停；解除后重新确认目标
ros2 param set /follow_supervisor emergency_stop true
ros2 param set /follow_supervisor emergency_stop false

# 目标丢失
ros2 param set /simulated_person force_lost true
ros2 param set /simulated_person force_lost false

# 检测存在，但没有可靠 XYZ
ros2 param set /simulated_person force_invalid_depth true
ros2 param set /simulated_person force_invalid_depth false

# 模拟视觉进程停止发布：最多约 0.5 秒后进入停车
ros2 param set /simulated_person pause_publishing true
ros2 param set /simulated_person pause_publishing false
```

停发测试期间 RViz 目标真值仍可以更新，控制节点不能利用真值继续行驶。也可以点击 Gazebo 暂停：ROS 仿真时间暂停，控制节点的墙上时间计时仍会在约 0.5 秒后发布零速度。恢复后等待新测量重新确认。

整个 `/person/*` 测量流停发时，最后的 `/person/visible` 值可能仍为 true，但其消息过期也会使控制输入“不可靠”。当前判定顺序下，状态通常为 `target_not_reliable`；`target_stale` 对应可见标志仍新鲜而 XYZ 过期等情况。检查停发时应确认输入更新已中断、速度归零及恢复后的动作，精确停车延迟需结合 CSV 的消息年龄与实际 odom 测量。

## 8. 保存记录

运动中的正常退出采用独立 `normal_exit` 记录：`run.sh normal_exit` 启动away运动，再由 `request_normal_exit.sh` 给工作空间中唯一的监督节点发送SIGINT，让Gazebo和记录节点继续运行。操作及判定见[正常退出测试](docs/normal-exit-test.md)。

每轮自动保存 CSV 到 `~/person_follow_ws/records/`。需要完整消息回放时，在启动本轮仿真前另开终端运行：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/record_bag.sh
```

结束时两边分别 `Ctrl+C`。bag 记录 `/clock`、TF、里程计、输入测量、状态及候选/最终速度。本阶段没有原始 RGB-D 图像。屏幕 MP4 可帮助看行为，但不是数值和时间戳记录的替代品。

结束本轮仿真后，可以集中收集已有记录，供宿主机直接分析：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/collect_records.sh
```

脚本将每个场景按启动时间最新的一份CSV及已有版本记录复制到共享项目的`records/incoming/`，原始文件仍保存在Ubuntu工作空间中。该目录已由Git忽略，收集内容只保存在本地。脚本会明确列出缺失的场景；文件收集不代表检查通过，仍需核对内容、运行时长及实际动作。

## 9. 参数修改

跟随距离通过同一启动参数同时传给官方服务器和 C++ 监督节点：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/run.sh stationary desired_distance:=1.8
```

其他参数见 `src/person_follow_bringup/config/following.yaml`。修改源码或 YAML 后重新执行 `scripts/build.sh`，再启动。监督节点的距离、速度上限、坐标系等参数只在启动时读取并设为只读，避免只修改官方服务器、导致两个节点距离设置不一致。`emergency_stop` 和三个模拟故障参数可以实时修改。

相机默认前向偏移 0.25 米、高度 1.0 米。模拟视野是控制测试的示意设置，不代表 D435i 的实际标定内参。调整相机安装位置时，必须同时调整 URDF 固定变换和模拟目标参数。

## 10. 后续推进

1. **完成当前仿真验收**：编译成功、目标跟随、靠近停车、转向、失效停车、恢复；保存参数和记录。
2. **接入真实视觉接口**：使用现有 Python 视觉包的 PointStamped/visible，先在车轮不运动的条件下验证时间戳、TF、坐标方向。相机若固定在桌上，不能假设它会随仿真车移动来验证完整跟随闭环。
3. **加入障碍物**：添加激光/深度障碍物输入、costmap、Nav2 规划和控制，再设计动态跟随目标更新。当前跟随服务器没有启用碰撞检测，不能靠它自动绕行。
4. **对接真实底盘**：接 C++ 底盘驱动、轮速/里程计、相机安装 TF，切换 `use_sim_time=false`；加入底盘自身的指令超时停车和实际急停。先架空轮子，再低速空场测试。
5. **特殊人员或手势**：保留视觉包的接口，后续扩展身份/行为识别；控制包不依赖 YOLO 的具体语言或模型版本。

更完整的里程碑和通过条件见 [推进计划](docs/roadmap.md)。

## 11. 学习资源

- [官方 Humble 跟随包 README 与源码](https://github.com/open-navigation/opennav_docking/tree/humble/opennav_following)：目标接口、Action 和服务器参数。
- [ROS 2 Humble：tf2 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)：理解光学坐标系、底盘坐标系和 odom。
- [ROS 2 Humble：C++ Action 客户端](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)：理解启动、取消和结果反馈。
- [Gazebo ROS 差速驱动插件官方源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)：轮间距、轮径、里程计和 TF。
- [Nav2 官方文档](https://docs.nav2.org/)：后续 costmap、规划器、控制器和碰撞监测；学习时注意选择与 Humble 对应的接口。

本仓库的源码、README 和测试记录当前只保存在本地；没有向 GitHub 推送。
