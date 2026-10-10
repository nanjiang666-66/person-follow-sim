# E1：真实D435i与C++控制的接口观察

实车约两天后到位。到货前先将D435i固定摆放，沿用已验证的YOLO26n-seg视觉包，核对真实测量的单位、左右方向、时间戳、失效语义与控制输出。

这套入口使用固定测试TF与零速度里程计，最终速度发布到 **/follow_preview/cmd_vel**。没有启动Gazebo、底盘驱动或模拟/person源。固定相机的测量不会随虚拟车移动，所以本项验证消息接口与控制反应；完整跟随闭环在相机随实车移动、真实odom接入后验证。

## 1. 编译新增观察节点

结束上一套仿真，在Ubuntu执行：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/build.sh
```

新增C++节点vision_preview_fixture与launch需要本次编译才能使用。宿主机已检查启动文件AST、manifest XML和Shell语法；没有ROS编译环境，**新增ROS C++节点的编译与实际联调仍需Ubuntu验证**。

## 2. 统一现实时间与ROS域

相机、视觉和观察控制均使用ROS_DOMAIN_ID=17、use_sim_time=false。相机或视觉原来若在其他域运行，先结束旧节点，再在下面的终端重新启动。

终端A，相机：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=17
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30 \
  use_sim_time:=false
```

这里沿用此前相机分辨率；若你已有通过验证的相机启动参数，可保持原参数并设置相同域和现实时间。

终端B，沿用分割测距模型：

```bash
source /opt/ros/humble/setup.bash
source ~/d435i_vision_venv/bin/activate
source ~/person_vision_ws/install/setup.bash
export ROS_DOMAIN_ID=17
ros2 run person_vision person_tracker --ros-args \
  -p model:=$HOME/person_vision_ws/models/yolo26n-seg.pt \
  -p segment_depth:=true -p pose_depth:=false \
  -p image_size:=416 -p confidence:=0.25 -p use_sim_time:=false
```

本阶段不更换已验证的视觉算法；权重仍使用Ubuntu已有模型文件。点选原来的跟踪对象，确认可靠深度正常。

终端C，启动C++观察控制：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/run_vision_preview.sh
```

脚本打印vision_preview_时间戳.csv路径，并检查当前域是否已有Gazebo、模拟目标或另一套跟随控制；发现时停止启动。源码中的控制算法没有更改，只有速度输出经ROS remap改到观察话题，初始化日志也显示实际发布话题。

## 3. 测试坐标和里程计

```text
follow_preview_odom
  └── follow_preview_base       固定测试底盘，odom位置和速度均为零
        └── camera_link        默认测试x=0米、height=1米，姿态为单位旋转
              └── ... -> camera_color_optical_frame  由RealSense驱动提供
```

fixture只连接测试底盘到相机根坐标，保留RealSense自己发布的彩色/深度内参外参和光学TF，避免给同一光学帧重复指定父坐标。

默认测试底盘的水平原点设在camera_link附近，测试高度1米只用于坐标表示，不是实际安装测量。底盘x向前、y向左；相机光学x向右、y向下、z向前。变换后的base.x应大体接近光学z，base.y应大体接近负的光学x；内外参的小平移仍由驱动保留。相机原始point.y上下变化不应被直接当成底盘左右偏移。

camera_link到测试base的姿态假设水平对准，未用它推断真实地面的安装角度。实际相机俯仰、朝向和安装位置在实车到位后测量并替换；测试TF和零odom必须在接实车时退出。

默认1.5米仍是到测试base的水平距离，包含横向分量，不是光学point.z单独的读数。需要临时改测试水平偏移时：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/run_vision_preview.sh \
  camera_x:=0.25 camera_height:=1.0
```

若相机实际根/光学frame_id不是默认值，按真实TF树指定camera_root_frame与camera_optical_frame。不要为消除TF报错修改测量原始采集时间戳。

## 4. 第一轮先检查接口

终端D：

```bash
source /opt/ros/humble/setup.bash
source ~/person_follow_ws/install/setup.bash
export ROS_DOMAIN_ID=17
ros2 topic info /person/target_point --verbose
ros2 topic info /follow_preview/cmd_vel --verbose
ros2 param get /follow_supervisor use_sim_time
timeout 5 ros2 topic echo /follow/state --once
timeout 5 ros2 topic echo /follow/target_base --once
timeout 5 ros2 topic echo /follow_preview/cmd_vel --once
```

期望：目标测量唯一发布者为d435i_person_tracker；观察速度唯一发布者为follow_supervisor；控制use_sim_time=false。有可靠深度和新鲜时间戳时完成确认，输出测试base坐标和观察速度。启动后尚未确认或目标未选中时零速度是正常结果。

若选中目标仍没有base点，先回传状态和完整 `Target TF unavailable` 警告。检查相机TF：

```bash
ros2 run tf2_ros tf2_echo follow_preview_base camera_color_optical_frame
```

不要同时运行模拟/person节点，也不要把/follow_preview/cmd_vel接到Gazebo或真实底盘。本阶段固定odom不响应观察指令，不具有车辆跟随的动力学效果。

## 5. 后续三个观察任务

2026-10-06，用户已回传第2–4节结果，基础话题、现实时间、测试base坐标和TF链已连通，可以继续本节。状态、坐标和速度是不同时间采样，暂不作为动态动作验收；下一轮保存连续CSV即可，不需等待机器人。

首份长记录之后，44秒短中央CSV已确认前后动作通过；仍约2.4%的可见采样原时间年龄超过0.5秒，时序连续性待定位。下一项按[左右动作与时间诊断](vision-timing-next-test.md)执行，不重复前后整项；完整结果见[CSV复核](vision-interface-review-20261006.md)。

第一轮接口正常后再做这些，无需重新测试整个视觉算法。

1. **距离方向**：人在中央，约2米站定、靠近到约1.2米、再远离到约2米，每处停留几秒。变换后的base.x应随远离增大；水平距离明显大于1.5米且方向对准时应有正前进指令，靠近后前进停止且不倒车，远离后恢复。
2. **左右方向**：人从图像左侧移到右侧。左侧原始x<0、测试base.y>0、通常正角速度；右侧相反。小偏角可能处于死区或到达容差，零角速度允许，不能据此单独判失败。
3. **失效**：目标遮挡或无可靠深度时visible=false，观察速度归零。随后正常恢复应重新确认。结束视觉节点而保留相机和观察控制时，测量过期也必须归零。

控制仍保留约0.5秒的测量/接收超时。若CPU推理或USB让图像采集到输出的时间过长，应先记录延迟和处理帧率，不放宽门限以获得运动指令。

## 6. 保存和分析

fixture每50ms记录一次CSV：原始光学XYZ、原始采集时间的年龄、消息接收年龄、visible、转换后的测试base XYZ及其原时间年龄、观察速度、控制状态和深度状态。未收到数据的标志为false，未收到消息的年龄写-1；旧坐标保留用于诊断，不能据数值还在CSV中认定当前有效。

没有重新给/person测量打时间戳；保留视觉包来自彩色图的header.stamp。固定测试odom每20ms使用现实时间发布，frame为follow_preview_odom，child为follow_preview_base。

结束观察控制后收集：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/collect_records.sh
```

vision_preview的CSV字段与仿真真值记录不同，不能使用常规check_recording.py判定。先发编译结果及第一轮检查输出，再结合实际CSV确定后续任务；实车到位后，另行确认底盘驱动接口、真实odom、安装TF和指令超时停车。
