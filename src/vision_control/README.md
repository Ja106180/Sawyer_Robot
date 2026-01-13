# vision_control 功能包

视觉控制功能包，使用USB摄像头进行人脸检测和手势识别。

## 功能概述

1. **人脸检测**：使用MediaPipe检测人脸并显示检测框
2. **手势识别**：
   - 左右上下滑动 → 对应箭头变绿（1秒后恢复白色）
   - 放大（张开） → 大圆变绿（1秒后恢复白色）
   - 缩小（闭合） → 小圆变绿（1秒后恢复白色）
3. **可视化界面**：
   - 显示摄像头画面
   - 人脸检测框
   - 右上角UI：4个方向箭头 + 中间一大一小两个圆

## 依赖

- ROS Noetic
- Python 3
- OpenCV
- MediaPipe

安装MediaPipe：
```bash
pip3 install mediapipe
```

## 使用方法

### 方法一：使用launch文件（推荐）

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch vision_control vision_control.launch
```

### 方法二：直接运行节点

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun vision_control vision_control_node.py
```

### 参数配置

可以通过launch文件修改参数：

```xml
<param name="camera_id" value="0" />
<param name="frame_width" value="640" />
<param name="frame_height" value="480" />
<param name="fps" value="30.0" />
<param name="swipe_threshold" value="30.0" />
<param name="swipe_frames" value="5" />
<param name="zoom_threshold" value="0.15" />
<param name="zoom_speed_threshold" value="0.05" />
<param name="ui_highlight_duration" value="1.0" />
```

## 操作说明

1. **启动系统**：运行launch文件后，会打开一个窗口显示摄像头画面
2. **人脸检测**：系统会自动检测人脸并显示绿色检测框
3. **手势识别**：
   - **滑动**：快速向左/右/上/下滑动手，对应箭头会变绿1秒
   - **放大**：从拳头突然张开手，大圆会变绿1秒
   - **缩小**：从张开突然握拳，小圆会变绿1秒
4. **退出**：按 'q' 键退出程序

## 参数说明

- `camera_id`: USB摄像头ID（默认0）
- `frame_width`: 图像宽度（默认640）
- `frame_height`: 图像高度（默认480）
- `fps`: 帧率（默认30.0）
- `swipe_threshold`: 滑动速度阈值，像素/帧（默认30.0）
- `swipe_frames`: 连续帧数确认滑动（默认5）
- `zoom_threshold`: 放大/缩小变化阈值（默认0.15）
- `zoom_speed_threshold`: 变化速度阈值（默认0.05）
- `ui_highlight_duration`: UI元素高亮持续时间，秒（默认1.0）

## 注意事项

1. 确保USB摄像头已连接
2. 如果摄像头ID不是0，请修改`camera_id`参数
3. 手势识别需要良好的光照条件
4. 滑动手势需要快速且连续的动作
5. 放大/缩小手势需要明显的张开/闭合动作

