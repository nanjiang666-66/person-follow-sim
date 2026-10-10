# 下一项：左右动作与时间诊断

2026-10-07的新CSV及同轮Timing日志已复核：右侧方向及失效时零指令已有证据，左侧有效停留不足，38–44秒存在可靠测距中断；诊断已实际启用，识别入口前图像年龄约0.14–0.22秒，再处理约0.05–0.07秒发布XYZ，偶发发布年龄超过0.5秒。先确认38–44秒的实际动作，再确定补测及旧帧积压优化范围，见[本轮复核](vision-interface-review-20261007.md)。下面保留诊断加载与测试方法，已启用诊断不需要再次更新代码。

前后动作已由44秒短CSV确认通过，无需重做前后。还需定位约2.4%的可见采样在等待更新时超过0.5秒的现象，下一次左右动作同时记录新增时间日志。

## 1. 加载新增诊断代码

这次需要更新识别程序：在观察控制终端和识别终端分别Ctrl+C，**相机终端保持运行**。更新只增加诊断，timing_debug默认false，作为启动设置读取。

在识别终端执行：

```bash
cp -a /mnt/hgfs/ubuntu-workspace/d435i_person_tracking/person_vision/. \
  ~/person_vision_ws/src/person_vision/person_vision/
cd ~/person_vision_ws
source /opt/ros/humble/setup.bash
source ~/d435i_vision_venv/bin/activate
colcon build --symlink-install --packages-select person_vision
source install/setup.bash
export ROS_DOMAIN_ID=17
ros2 run person_vision person_tracker --ros-args \
  -p model:=$HOME/person_vision_ws/models/yolo26n-seg.pt \
  -p segment_depth:=true -p pose_depth:=false \
  -p image_size:=416 -p confidence:=0.25 \
  -p use_sim_time:=false -p timing_debug:=true
```

复制范围是person_vision模块目录，不包含.git。重新单击测试对象，确认绿框和可靠XYZ出现。若此前另有通过验证的视觉参数，继续保留那些参数，并增加timing_debug:=true。

观察控制终端再启动：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/run_vision_preview.sh
```

## 2. 左右动作

人在约1.8–2米处，保持大致相同的前后距离：

1. 中央停5秒。
2. 移到图像左侧并停5–10秒；测量raw_x<0，通常base.y>0，稳定时cmd_w应为正。
3. 回中央停5秒。
4. 移到右侧并停5–10秒；raw_x>0，通常base.y<0，稳定时cmd_w应为负。
5. 回中央停5秒。

偏移足够看出符号，但保持人在相机画面内。小偏角有死区，较大偏角先转向、前进归零是允许的；动作刚切换时角速度受变化率与目标滤波影响，检查停留后的稳定片段。

结束观察控制后收集CSV：

```bash
bash /mnt/hgfs/ubuntu-workspace/person_follow_sim/scripts/collect_records.sh
```

回传收集输出、测试时识别终端的1–2组Last…FPS和Timing日志，以及稳定左侧/右侧时的观察结果。随后进行遮挡/识别停发测试。

## 3. 时间日志含义

每5秒额外打印一条 `Timing ms (mean/max)`，数字为该窗口已观察样本的平均/最大毫秒，n为样本数。

| 字段 | 含义 |
|---|---|
| color_rx / depth_rx | 识别进程开始执行对应订阅回调时，图像原始时间戳的年龄 |
| color_entry / depth_entry | 同步器交出一对图像、完整处理开始时的图像年龄 |
| xyz_pub | 即将发布可信XYZ时，原始彩色图时间戳的年龄 |
| pair_gap | 被处理的彩色与对齐深度图时间戳绝对差 |
| compute_to_pub | 同步处理开始到准备发布XYZ的墙上耗时，尚未包含publish调用内部及显示尾段 |

color_rx已经包含本进程回调等待，不能单凭它认定相机或USB慢；不同字段也可能有不同样本数，均值差不能直接作为同一帧的精确分解。新CSV提供接收端的总年龄，配合这些日志判断是入口前积压、同步等待、计算，还是发布后延迟。

没有可信XYZ时xyz_pub的n可以为0。日志不修改原始header.stamp、位置或可见性；原时间戳和无效深度不发布坐标已由回调测试检查。诊断最多保留每项512个样本，默认关闭。本地涉及8个人体节点回调检查（含2项新增诊断检查）、7个分割回调、7个姿态回调检查通过；这些测试使用替代ROS和模型接口，Ubuntu真实运行的诊断输出仍需验证。
