## yolo_package

YOLO 推理 ROS 节点，支持 USB 摄像头和机械臂头部摄像头。

### 安装依赖

运行前请先在工作空间根目录执行：

```bash
cd ~/catkin_ws
source devel/setup.bash
```

### 启动 YOLO 推理节点

**默认配置**：程序已默认设置为使用机械臂头部摄像头和指定的模型路径。

```bash
rosrun yolo_package yolo_inference.py
```

就是这么简单！程序会自动：
- 使用机械臂头部摄像头（source=1）
- 加载模型：`/home/mycar/YOLO/ultralytics-8.3.163/yolov8n.pt`
- 自动设置摄像头曝光和增益为自动模式（auto-exposure 和 auto-gain）
- 弹出窗口显示 YOLO 识别结果

### 高级参数（可选）

如果需要自定义，可通过 `rosrun ... _param:=value` 设置：

- `model`：YOLO 权重文件路径，默认 `/home/mycar/YOLO/ultralytics-8.3.163/yolov8n.pt`
- `source`：视频源类型
  - `1`：机械臂头部摄像头（默认）
  - `0`：USB 摄像头
- `stream`：是否持续读取流，默认 `true`
- `window_name`：OpenCV 显示窗口名称，默认 `YOLO Inference`

### 示例

使用 USB 摄像头（而不是默认的机械臂头部摄像头）：

```bash
rosrun yolo_package yolo_inference.py _source:=0
```

### 使用方法

- 运行后会自动弹出窗口显示 YOLO 识别结果
- 按 `q` 键可退出推理
- 按 `Ctrl+C` 也可安全退出

