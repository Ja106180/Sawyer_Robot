## arm_follow 功能包

机械臂跟随功能包，通过YOLO Pose识别人体姿态，控制Sawyer机械臂跟随人体右手。

### 功能概述

1. **订阅YOLO关键点数据**：从 `/yolo_pose/keypoints` 话题接收人体关键点
2. **筛选最大的人**：自动选择检测框面积最大的人作为跟踪目标
3. **判断右手位置**：计算右手在图像中的角度（0-180度，从底部到顶部）
4. **跟随条件**：只有当右手在50-150度范围内时才开始跟随
5. **平滑跟踪**：使用指数移动平均避免抖动
6. **安全机制**：
   - 检测突变（角度变化>90度）时保持当前位置
   - 检测中断或目标消失时保持上一时刻位置
7. **控制机械臂**：控制Sawyer右臂的 J1（肩俯仰）与 J3（肘弯曲），其余关节保持预设姿态

### 使用方法

#### 方法一：使用launch文件（推荐，一条命令）

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch arm_follow arm_follow.launch
```

启动后会自动执行一次姿态初始化（等同于依次运行 gnd/head/j2/j4/j5/j6 脚本），完成后才开始跟随机制。该流程会启动：
- YOLO姿态识别节点（Python）
- C++ 手臂跟随节点（仅控制 right_j1 与 right_j3，其他关节保持初始化后的状态）

#### 方法二：分别启动（调试时使用）

**1. 启动YOLO识别节点**

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun yolo_package yolo_inference.py
```

**2. 启动机械臂跟随节点**

在新的终端中：

```bash
cd ~/catkin_ws
source devel/setup.bash
# 先固定底座与头部
rosrun arm_follow posture_init.py --j0 0.0 --head -1.5 --j2 0.0 --j4 0.0 --j5 0.0 --j6 0.0

# 启动C++跟随节点（此时 YOLO 已在另一个终端运行）
rosrun arm_follow arm_follow_node
```

#### Launch文件参数配置

如果需要修改参数，可以在启动时覆盖默认值：

```bash
# 调整平滑与速度感知
roslaunch arm_follow arm_follow.launch smoothing_factor:=0.7 speed_adapt_gain:=0.2

# 修改YOLO模型路径
roslaunch arm_follow arm_follow.launch yolo_model:=/home/mycar/YOLO/ultralytics-8.3.163/yolo11n-pose.pt

# 调整预设姿态
roslaunch arm_follow arm_follow.launch preset_head:=-1.2 preset_j2:=0.4
```

可用的launch参数：
- `yolo_model`: YOLO模型路径（默认：yolov8n-pose.pt）
- `smoothing_factor` / `smoothing_min` / `smoothing_max`: 平滑相关参数
- `confidence_threshold`: 关键点置信度阈值
- `speed_adapt_gain`: 手部速度对平滑的影响
- `prediction_horizon`: 速度外推时长（秒）
- `max_delta_per_cycle`: 单周期最大的关节变化量（弧度）
- `j1_smoothing_factor` / `j1_max_delta` / `j1_snap_threshold`: j1 专用的平滑/角度限制与“接近目标时吸附阈值”，用于控制大臂跟随与平滑
- `j3_smoothing_factor` / `j3_max_delta` / `j3_snap_threshold`: j3 专用的平滑/角度限制与“接近目标时吸附阈值”，用于提高小臂响应且避免末端甩动
- `j1_min` / `j1_max`、`j3_min` / `j3_max`: right_j1 / right_j3 的运动范围
- `j3_scale` / `j3_offset`: 控制人类肘角到 right_j3 的线性映射（`j3 ≈ neutral + offset - scale * elbow_angle`）
- `shoulder_rel_min` / `shoulder_rel_max`: 将“肩→肘 方向向量纵向分量”映射到 j1 的参数（默认 -1.5~1.5，可根据需要调整）
- `shoulder_gain`: 向量纵向分量的放大系数（默认 1.5，用来控制灵敏度）
- `shoulder_metric_alpha`: j1 专用的低通滤波系数（默认 0.3，越大越平滑）
- `preset_j0` / `preset_j1` / `preset_j2` / `preset_j3` / `preset_j4` / `preset_j5` / `preset_j6` / `preset_head`: 启动时的固定姿态

或者直接在launch文件中修改 `launch/arm_follow.launch` 文件的默认参数值。

### 参数配置

可以通过ROS参数或launch覆盖上述全部参数；使用 `rosparam get /arm_follow_node` 可以查看当前值。

### 姿态初始化 & 快捷脚本

launch 会自动调用 `posture_init.py` 完成以下操作：底座、头部以及 j1~j6 复位，并在成功后设置 `/arm_follow/posture_ready=true` 供 C++ 节点检测。如果想手工复位或更改姿态，也可直接运行脚本：

```bash
# 设置头部角度（弧度）
rosrun arm_follow head_set.py --angle 0.3

# 设置底座关节 right_j0 角度（弧度）
rosrun arm_follow gnd_set.py --angle -0.8

# 单独调节各个手臂关节
rosrun arm_follow j1_set.py --angle 0.2   # right_j1
rosrun arm_follow j2_set.py --angle 0.5   # right_j2
rosrun arm_follow j3_set.py --angle -0.3  # right_j3
rosrun arm_follow j4_set.py --angle -0.2  # right_j4
rosrun arm_follow j5_set.py --angle 0.4   # right_j5
rosrun arm_follow j6_set.py --angle 1.0   # right_j6

# 控制爪子（末端执行器）
rosrun arm_follow j_zha.py --open        # 打开爪子
rosrun arm_follow j_zha.py --close       # 关闭爪子
rosrun arm_follow j_zha.py --position 0.5  # 设置爪子位置（0.0=关闭，1.0=打开）
```

- 所有脚本都提供 `--speed`、`--timeout`、`--threshold` 等可选参数，具体可通过 `--help` 查看
- 每个脚本都会自动将角度裁剪到安全范围，避免误操作

### 工作原理

1. **关键点解析**：YOLO Pose 直接输出带语义的右肩/右肘/右腕（COCO 索引 6/8/10），只要置信度 > 阈值即可认定为“右手臂”
2. **预测与速度自适应**：
   - 维护最近的关键点历史，基于速度做短时线性预测，降低延迟
   - 根据手部速度动态调整平滑系数，高速动作时更灵敏
3. **多关节协调**：
   - 仅控制 right_j1（肩俯仰）与 right_j3（肘弯曲），其中 j1 由“肩→肘方向向量的纵向分量”映射，j3 通过判断手腕相对肘的朝向来决定正负方向
   - 其他关节保持启动时姿态，避免额外抖动
4. **安全机制 / 碰撞防护**：
   - 每个关节都有软限制与最大单周期变化量（`max_delta_per_cycle`）
   - 检测不到目标或置信度不足时维持上一帧姿态，不再抖动

### 可视化

YOLO窗口会显示：
- 人体检测框
- 骨架关键点
- 关键点连线

### 注意事项

1. 确保机械臂已启动并初始化
2. 启动 launch 时会自动复位底座/头部/j2/j4/j5/j6，如需手动调整可提前使用对应脚本
3. 站在机械臂头部摄像头前方，右手臂保持在摄像头视野的同一平面
4. 尽量避免突然离开画面或剧烈超出平面范围，虽然有预测与限幅机制，但过大动作仍可能触发保护

### 故障排查

- **机械臂不跟随**：
  - 检查 `/yolo_pose/keypoints` 话题是否有数据：`rostopic echo /yolo_pose/keypoints`
  - 检查右手是否在50-150度范围内
  - 查看日志输出确认跟随状态

- **机械臂抖动**：
  - 增大 `smoothing_factor` 参数（例如0.8或0.9）
  - 检查视频流延迟

- **检测不到人**：
  - 确保光线充足
  - 确保人完全在摄像头视野内
  - 检查YOLO窗口是否正常显示

