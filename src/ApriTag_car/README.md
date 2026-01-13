# ApriTag_car - AprilTag 小车控制系统

## 功能概述

使用 AprilTag 36h11 识别小车位置和朝向，实现小车在 3 个目标点之间循环运动。

## 系统架构

1. **AprilTag 检测节点** (`apriltag_detection.py`)
   - 使用 Python `apriltag` 库检测图像中的 AprilTag
   - 使用相机内参计算 tag 位姿（相机坐标系）
   - 发布位姿到 `/apriltag_car/tag_pose`

2. **平面坐标映射标定** (`plane_calibration.py`)
   - 手动将 tag 放到 4 个已知平面坐标点
   - 记录相机坐标 -> 平面坐标的对应关系
   - 计算单应性矩阵 H，保存到 `config/plane_calibration.yaml`

3. **控制节点** (`apriltag_control_node.cpp`)
   - 订阅 tag 位姿，转换为平面坐标
   - 使用虚拟弹簧控制算法
   - 实现 A→B→C→A 循环运动

## 安装依赖

```bash
pip install apriltag
```

## 使用步骤

### 1. 相机内参标定（如果还没做）

使用 `my_car_yolo` 包中的 `camera_calibration.py`：

```bash
rosrun my_car_yolo camera_calibration.py
```

- 按 `c` 拍摄标定图片（至少 15-25 张）
- 按 `s` 开始标定计算
- 结果保存到 `my_car_yolo/config/camera_intrinsics.yaml`

**注意**：标定板参数 `square_size = 1.5` mm（保持不变）

### 2. 平面坐标映射标定

```bash
roslaunch ApriTag_car plane_calibration.launch
```

**操作步骤**：
1. 将 AprilTag 放在已知平面坐标点上（4个点）
2. 按**空格键**记录当前点对
3. 重复步骤1-2，直到记录完4个点
4. 按 `s` 保存标定结果
5. 按 `q` 退出

**默认4个标定点**（可在 `plane_calibration.py` 中修改）：
- (0.0, 0.0) - 原点（图像左上角对应）
- (0.5, 0.0) - X+ 方向 50cm
- (0.5, 0.5) - X+ Y+ 方向 50cm
- (0.0, 0.5) - Y+ 方向 50cm

标定结果保存到 `config/plane_calibration.yaml`

### 3. 运行控制系统

```bash
roslaunch ApriTag_car apriltag_car.launch
```

**功能**：
- 小车自动在 3 个目标点之间循环：A→B→C→A→...
- 使用虚拟弹簧控制算法
- 到达目标点后自动切换到下一个点

**默认3个目标点**（可在 `apriltag_control_node.cpp` 中修改）：
- A: (0.3, 0.3) 米
- B: (0.7, 0.3) 米
- C: (0.5, 0.7) 米

## 参数说明

### AprilTag 参数
- `tag_size`: Tag 尺寸（米），默认 0.05（5cm）

### 控制参数（在 launch 文件中）
- `arrival_threshold_meters`: 到达距离阈值（米），默认 0.08
- `linear_kp`: 线速度比例系数，默认 0.5
- `angular_kp`: 角速度比例系数，默认 0.35
- `angular_kd`: 角速度阻尼系数，默认 0.15
- `angular_dir`: 转向方向（+1/-1），默认 -1.0
- `max_linear_speed`: 最大线速度（m/s），默认 0.12
- `max_angular_speed`: 最大角速度（rad/s），默认 0.4

## 坐标系约定

- **平面坐标原点** = 图像左上角
- **X 轴** = 图像向右（正方向）
- **Y 轴** = 图像向下（正方向）

## 注意事项

1. **AprilTag 安装**：
   - Tag 中心应与小车中心对齐
   - Tag 朝向：先测试，如果朝向不对再调整 `angular_dir` 参数

2. **相机图像话题**：
   - 默认订阅 `/camera/image_raw`
   - 如果话题名不同，在 launch 文件中修改 `image_topic` 参数

3. **修改目标点坐标**：
   - 编辑 `src/apriltag_control_node.cpp` 中的 `target_points_` 数组
   - 重新编译：`catkin_make ApriTag_car`

4. **修改标定点坐标**：
   - 编辑 `scripts/plane_calibration.py` 中的 `plane_points` 列表
   - 重新运行标定程序

## 文件结构

```
ApriTag_car/
├── scripts/
│   ├── apriltag_detection.py      # AprilTag 检测节点
│   └── plane_calibration.py       # 平面坐标映射标定脚本
├── src/
│   └── apriltag_control_node.cpp  # 控制节点（C++）
├── launch/
│   ├── apriltag_car.launch        # 主启动文件
│   └── plane_calibration.launch    # 标定启动文件
├── config/
│   ├── camera_intrinsics.yaml      # 相机内参（从 my_car_yolo 复制）
│   └── plane_calibration.yaml      # 平面标定结果（标定后生成）
└── CMakeLists.txt
```

## 故障排查

1. **检测不到 AprilTag**：
   - 检查 tag 是否在相机视野内
   - 检查 tag 尺寸参数是否正确（5cm）
   - 检查相机内参是否正确

2. **小车运动方向不对**：
   - 调整 `angular_dir` 参数（+1 或 -1）
   - 检查 tag 朝向是否正确

3. **到达判定不准确**：
   - 调整 `arrival_threshold_meters` 参数
   - 检查平面标定是否准确

