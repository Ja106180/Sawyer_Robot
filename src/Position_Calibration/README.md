# Position_Calibration - 机械臂视觉抓取标定功能包

## 功能说明

本功能包用于标定机械臂头部摄像头与桌面平面的单应性矩阵，实现从图像像素坐标到机械臂坐标的转换。

## 功能特点

- 使用YOLOv8模型检测纸盒
- 交互式标定流程，收集5个点对
- 自动计算单应性矩阵
- 保存标定结果到配置文件

## 使用方法

### 1. 编译功能包

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 运行标定脚本

**方法1：使用launch文件**
```bash
roslaunch Position_Calibration calibration.launch
```

**方法2：直接运行脚本**
```bash
rosrun Position_Calibration position.py
```

### 3. 标定流程

1. **启动脚本后**：
   - 脚本会自动初始化机械臂到固定位置
   - 等待摄像头图像

2. **收集点对（重复5次）**：
   - 在摄像头视野的不同位置放置纸盒（建议：4个边角区域 + 中心区域）
   - 系统会自动检测纸盒，显示检测框和中心点
   - 按 `y` 键确认识别结果
   - 手动控制机械臂，将j6移动到纸盒中心正上方
   - 按 `r` 键记录点对
   - 系统会自动将机械臂初始化到固定位置，准备下一个点

3. **标定完成**：
   - 收集够5个点对后，自动计算单应性矩阵
   - 显示标定误差
   - 保存结果到 `config/calibration.yaml`

### 4. 键盘控制

- `y` - 确认识别结果
- `r` - 记录当前点对
- `c` - 清除最后一个点对
- `q` - 退出标定

## 参数配置

可以通过launch文件或ROS参数设置以下参数：

- `model`: YOLO模型路径（默认：`/home/mycar/YOLO/ultralytics-8.3.163/yolov8n_landmark.pt`）
- `camera_topic`: 摄像头图像话题（默认：`/io/internal_camera/head_camera/image_rect_color`）
- `config_path`: 标定结果保存路径（默认：`$(find Position_Calibration)/config/calibration.yaml`）
- `table_height`: 桌面高度，单位米（默认：`0.746`）
- `conf_threshold`: YOLO检测置信度阈值（默认：`0.5`）

## 标定结果

标定结果保存在 `config/calibration.yaml` 文件中，包含：

- `homography_matrix`: 3×3单应性矩阵
- `table_height`: 桌面高度
- `calibration_points`: 标定点对数据（用于验证）
- `calibration_date`: 标定时间戳

## 注意事项

1. 标定前确保机械臂和摄像头正常工作
2. 标定时确保光照条件良好
3. 纸盒应放置在桌面不同位置，覆盖摄像头视野的主要区域
4. 每次记录点对前，确保机械臂j6准确移动到纸盒中心正上方
5. 标定完成后，检查标定误差，如果误差过大（>2cm），建议重新标定

## 依赖

- ROS (Noetic)
- Python 3
- OpenCV (cv2)
- NumPy
- Ultralytics (YOLOv8)
- intera_interface (Sawyer机械臂接口)
- cv_bridge

## 文件结构

```
Position_Calibration/
├── scripts/
│   └── position.py          # 标定脚本
├── config/
│   └── calibration.yaml     # 标定结果配置文件
├── launch/
│   └── calibration.launch   # 启动文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

