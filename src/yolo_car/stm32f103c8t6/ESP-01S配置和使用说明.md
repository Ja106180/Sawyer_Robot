# ESP-01S WiFi 配置和与 STM32F103 通信说明

## 一、ESP-01S 连接 WiFi 的方法

ESP-01S 需要通过 **AT 命令**来配置 WiFi。有两种方式：

### 方法1：使用 USB 转串口模块直接配置（推荐，首次配置）

#### 1.1 硬件连接
```
ESP-01S          USB转串口模块
VCC  →  3.3V
GND  →  GND
TX   →  USB转串口的 RX
RX   →  USB转串口的 TX
CH_PD → 3.3V
RST  → 3.3V（或悬空）
IO0  → 悬空（不接）
IO2  → 悬空（不接）
```

#### 1.2 使用串口工具发送 AT 命令

在 Ubuntu 中可以使用以下工具：

**方法A：使用 `minicom`**
```bash
# 安装 minicom
sudo apt install minicom

# 查看串口设备（通常是 /dev/ttyUSB0 或 /dev/ttyACM0）
ls /dev/ttyUSB* /dev/ttyACM*

# 打开串口（替换 /dev/ttyUSB0 为你的实际设备）
sudo minicom -D /dev/ttyUSB0 -b 115200
```

**方法B：使用 `screen`**
```bash
# 安装 screen（通常已安装）
sudo apt install screen

# 打开串口
sudo screen /dev/ttyUSB0 115200
```

**方法C：使用 `picocom`**
```bash
# 安装 picocom
sudo apt install picocom

# 打开串口
sudo picocom -b 115200 /dev/ttyUSB0
```

#### 1.3 AT 命令配置步骤

打开串口工具后，依次发送以下命令（每发送一条命令后，ESP-01S 会回复 `OK`）：

```bash
AT                          # 测试连接，应该回复 OK
AT+RST                      # 重启模块，等待几秒
AT+CWMODE=1                 # 设置为 STA 模式（客户端模式）
AT+CWJAP="你的WiFi名称","WiFi密码"    # 连接 WiFi，等待连接成功（可能需要10-30秒）
AT+CIPMODE=1                # 设置为透传模式
AT+CIPSTART="TCP","192.168.31.64",11411    # 连接到 PC 的 rosserial TCP 服务器
AT+CIPSEND                  # 进入透传模式（此时会显示 ">" 提示符）
```

**重要说明**：
- 将 `"你的WiFi名称"` 和 `"WiFi密码"` 替换为实际的 WiFi 名称和密码
- 将 `192.168.31.64` 替换为你的 PC IP 地址（运行 ROS 的机器）
- 端口 `11411` 是 rosserial TCP 服务器的默认端口
- 如果 WiFi 名称或密码包含特殊字符，需要用转义字符

**退出串口工具**：
- `minicom`: 按 `Ctrl+A`，然后按 `X`，选择 `Yes`
- `screen`: 按 `Ctrl+A`，然后按 `K`，选择 `Yes`
- `picocom`: 按 `Ctrl+A`，然后按 `Ctrl+X`

#### 1.4 保存配置（可选）

ESP-01S 默认会保存 WiFi 配置，下次上电会自动连接。如果需要手动保存：

```bash
AT+SAVETRANSLINK=1,"192.168.31.64",11411,"TCP"    # 保存透传配置
```

### 方法2：通过 STM32 转发 AT 命令（需要额外代码支持）

如果你想让 STM32 帮助配置 ESP-01S，需要：
1. STM32 接收串口命令
2. STM32 转发给 ESP-01S
3. ESP-01S 的回复通过 STM32 转发回来

**注意**：当前代码**不支持**这种方式，建议使用方法1直接配置。

## 二、ESP-01S 和 STM32F103 的连接方式

### 2.1 硬件连接（已完成）

按照 `ESP-01S接线说明.md` 中的接线方式连接：
- STM32 PA9 (TX) → ESP-01S RX
- STM32 PA10 (RX) ← ESP-01S TX
- 共地、共电源等

### 2.2 软件通信流程

```
┌─────────────┐     串口      ┌──────────┐     WiFi      ┌──────────┐
│  STM32F103  │ ←──────────→ │ ESP-01S  │ ←──────────→ │   PC     │
│             │  (USART1)    │ (透传)    │  (TCP)       │ (ROS)    │
│ rosserial   │              │          │              │          │
│ client      │              │          │              │          │
└─────────────┘              └──────────┘              └──────────┘
```

**工作流程**：
1. **STM32 端**：运行 rosserial client，通过 USART1 发送/接收数据
2. **ESP-01S 端**：作为透传桥接，将串口数据通过 WiFi TCP 转发
3. **PC 端**：运行 rosserial TCP 服务器，接收/发送数据给 ROS

### 2.3 通信协议

- **物理层**：USART1，115200 波特率，8N1（8位数据，无校验，1位停止位）
- **传输层**：TCP/IP（ESP-01S 负责）
- **应用层**：rosserial 协议（STM32 和 PC 都使用）

## 三、完整使用流程

### 3.1 首次配置 ESP-01S（只需一次）

1. 使用 USB 转串口模块连接 ESP-01S
2. 发送 AT 命令配置 WiFi 和 TCP 连接（见方法1）
3. 配置完成后，断开 USB 转串口模块

### 3.2 正常使用流程

1. **连接硬件**：
   - ESP-01S 按照接线说明连接到 STM32
   - 确保所有电源和信号线连接正确

2. **启动 ROS**：
   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   roslaunch yolo_car yolo_car.launch
   ```
   这会启动：
   - `yolo_car_detection`：YOLO 检测节点
   - `yolo_car_control`：控制节点
   - `rosserial_tcp`：TCP 服务器（监听 11411 端口）

3. **上电 STM32**：
   - 给 STM32 上电
   - ESP-01S 会自动连接 WiFi（如果之前已配置）
   - ESP-01S 会自动连接到 PC 的 TCP 服务器
   - STM32 通过 rosserial 与 ROS 建立连接

4. **验证连接**：
   - 查看 ROS 终端，应该能看到 `rosserial_tcp` 显示 "Client connected"
   - 或者运行：
     ```bash
     rostopic list
     ```
     应该能看到 `/yolo_car/cmd_vel` 和 `/yolo_car/servo_angle` 等话题

## 四、故障排查

### 4.1 ESP-01S 无法连接 WiFi

**检查项**：
- WiFi 名称和密码是否正确
- WiFi 是否为 2.4GHz（ESP-01S 不支持 5GHz）
- WiFi 信号强度是否足够
- ESP-01S 是否已正确配置为 STA 模式（`AT+CWMODE=1`）

**解决方法**：
```bash
AT+CWMODE=1                 # 确保是 STA 模式
AT+CWLAP                    # 扫描可用 WiFi，确认能看到你的 WiFi
AT+CWJAP="WiFi名称","密码"   # 重新连接
```

### 4.2 ESP-01S 无法连接到 PC

**检查项**：
- PC IP 地址是否正确（运行 `hostname -I` 查看）
- PC 防火墙是否阻止了 11411 端口
- `rosserial_tcp` 是否已启动
- ESP-01S 和 PC 是否在同一 WiFi 网络

**解决方法**：
```bash
# 在 PC 上查看 IP 地址
hostname -I

# 检查 rosserial_tcp 是否在运行
rosnode list | grep rosserial

# 测试 TCP 连接（在另一个终端）
telnet 192.168.31.64 11411
```

### 4.3 STM32 无法与 ROS 通信

**检查项**：
- ESP-01S 是否已连接到 WiFi
- ESP-01S 是否已连接到 PC 的 TCP 服务器
- STM32 和 ESP-01S 的串口连接是否正确（TX/RX 是否交叉）
- 波特率是否为 115200

**解决方法**：
1. 检查 ESP-01S 状态：
   ```bash
   # 通过 USB 转串口连接 ESP-01S，发送：
   AT+CWJAP?                # 查看当前连接的 WiFi
   AT+CIPSTATUS             # 查看 TCP 连接状态
   ```

2. 检查 STM32 串口：
   - 确认 PA9/PA10 连接正确
   - 确认代码中波特率设置为 115200

3. 检查 ROS 端：
   ```bash
   # 查看 rosserial_tcp 日志
   rosnode info /rosserial_tcp
   ```

### 4.4 连接不稳定

**可能原因**：
- 电源不稳定（ESP-01S 需要足够的电流）
- WiFi 信号弱
- 串口连接不良

**解决方法**：
- 使用外部 3.3V 稳压模块给 ESP-01S 供电
- 确保 ESP-01S 和路由器距离不要太远
- 检查所有连接是否牢固

## 五、常用 AT 命令参考

```bash
AT                          # 测试连接
AT+RST                      # 重启模块
AT+GMR                      # 查看版本信息
AT+CWMODE?                  # 查看当前模式（1=STA, 2=AP, 3=STA+AP）
AT+CWMODE=1                 # 设置为 STA 模式
AT+CWLAP                    # 扫描可用 WiFi
AT+CWJAP?                   # 查看当前连接的 WiFi
AT+CWJAP="SSID","密码"      # 连接 WiFi
AT+CWQAP                    # 断开 WiFi
AT+CIPSTATUS                # 查看 TCP 连接状态
AT+CIPSTART="TCP","IP",端口  # 建立 TCP 连接
AT+CIPMODE=1                # 设置为透传模式
AT+CIPSEND                  # 进入透传模式
AT+SAVETRANSLINK=1,"IP",端口,"TCP"  # 保存透传配置
```

## 六、总结

1. **首次配置**：使用 USB 转串口模块 + AT 命令配置 ESP-01S 的 WiFi 和 TCP 连接
2. **正常使用**：ESP-01S 作为透传桥接，STM32 通过串口与 ESP-01S 通信，ESP-01S 通过 WiFi TCP 与 PC 通信
3. **通信流程**：STM32 (USART1) ↔ ESP-01S (透传) ↔ PC (TCP) ↔ ROS (rosserial)

配置完成后，ESP-01S 会自动保存配置，下次上电会自动连接，无需重复配置。

