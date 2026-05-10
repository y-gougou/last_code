# R550PLUS 三全向轮机器人 ROS 包

## 概述

`turn_on_wheeltec_robot` 包含两条可并行使用的能力：

- **传感器数据采集**：13通道底盘数据 + 3通道电流数据，支持 CSV 采集和故障标签
- **Web 控制 + 定速巡航**：浏览器控制机器人，支持 PID 闭环巡航定速

---

## 数据来源

本系统有 **两个独立串口数据源**：

| 数据源 | 串口 | 话题 | 通道数 | 频率 |
|--------|------|------|--------|------|
| 底盘控制器 | `/dev/ttyCH343USB0` | `/odom`, `/imu`, `/PowerVoltage` | 13通道 | 20Hz |
| 电流传感器 | `/dev/ttyUSB0` | `/current_data` | 3通道 | 以实测为准 |

**内部采样频率**：IMU 芯片 MPU6050 内部采样 100Hz，但 ROS 发布频率为 20Hz。

**13通道底盘数据**：
- 位置：x, y, z（里程计）
- 速度：vx, vy, vz
- 加速度：ax, ay, az（IMU）
- 角速度：gx, gy, gz（IMU）
- 电压：voltage

**3通道电流数据**：
- current0, current1, current2

---

## 系统结构

```
串口设备 1 (ttyCH343USB0)          串口设备 2 (ttyUSB0)
        │                                    │
        ▼                                    ▼
┌───────────────┐                  ┌─────────────────┐
│wheeltec_robot │                  │ current_reader  │
│    _node      │                  │      .py        │
│    (C++)      │                  └────────┬────────┘
└───────┬───────┘                           │
        │  /odom /imu /PowerVoltage         │  /current_data
        └───────────────────┬───────────────┘
                            │
                   ┌────────▼────────┐
                   │ data_collector  │
                   │      .py        │
                   │ /odom触发写CSV   │
                   │ 最新值缓存填充   │
                   └─────────────────┘

浏览器 WebSocket
        │
        ▼
  rosbridge_websocket (ws://0.0.0.0:9090)
        │
        ▼
  /cmd_vel_web + /web/cruise_cmd + /web/cruise_enable + /web/heartbeat + /web/estop
        │
        ▼
  cmd_vel_web_adapter.py (安全保护 + PID巡航)
        │
        ▼
      /cmd_vel → wheeltec_robot_node
```

---

## 故障标签说明

| 标签 | 名称 | 说明 |
|------|------|------|
| 0 | normal | 正常状态 |
| 1 | drive_fault | 驱动异常（单轮堵转） |
| 2 | wheel_slip | 轮子打滑/损坏 |
| 3 | shaft_eccentric | 电机轴偏心 |

---

## 数据采集启动方式

采集时建议三个终端分开启动，避免底盘节点、电流读取节点和 Web 控制节点重复启动。

```bash
# 终端 1：启动底盘控制节点，发布 /odom、/imu、/PowerVoltage，并接收 /cmd_vel
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch

# 终端 2：启动电流读取和 CSV 数据采集
roslaunch turn_on_wheeltec_robot data_collector.launch \
  fault_label:=0 \
  fault_name:=normal \
  fault_mode:=0 \
  run_id:=normal_001 \
  motion_mode:=straight_0.5ms

# 终端 3：启动 Web 控制，禁用重复的数据采集节点
roslaunch turn_on_wheeltec_robot web_control.launch \
  start_base:=false \
  start_current_reader:=false \
  start_data_collector:=false
```

CSV 默认保存到：

```bash
/home/wheeltec/R550PLUS_data_collect/log/
```

### 采集命名规范

每个 CSV 文件内部会记录 `run_id`、`fault_label`、`fault_name`、`fault_mode`、`motion_mode`。建议每个类别至少采集 5 个独立 run：

```bash
normal_001.csv
normal_002.csv
drive_fault_001.csv
wheel_slip_001.csv
shaft_eccentric_001.csv
encoder_fault_001.csv
```

### 各类别启动命令

```bash
# 0 normal：正常
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0 fault_name:=normal fault_mode:=0 run_id:=normal_001 motion_mode:=straight_0.5ms

# 1 drive_fault：单轮驱动异常/驱动能力下降
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=1 fault_name:=drive_fault fault_mode:=1 run_id:=drive_fault_001 motion_mode:=straight_0.5ms

# 2 wheel_slip：轮子打滑
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=2 fault_name:=wheel_slip fault_mode:=0 run_id:=wheel_slip_001 motion_mode:=straight_0.5ms

# 3 shaft_eccentric：机械偏心/轴偏心
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=3 fault_name:=shaft_eccentric fault_mode:=0 run_id:=shaft_eccentric_001 motion_mode:=straight_0.5ms

# 4 encoder_fault：编码器异常，可选
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=4 fault_name:=encoder_fault fault_mode:=3 run_id:=encoder_fault_001 motion_mode:=straight_0.5ms
```

### 标准采集流程

标准工况暂定为 `0.5 m/s` 匀速直线运动：

```bash
# 0-5s 静止，记录基线
# 5-65s 发布 0.5 m/s 直线速度
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  -r 20

# 65-70s 停止
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  -1
```

采集足够时间后在终端 2 按 `Ctrl+C` 停止。

### ROS 端测试检查

启动后先检查话题是否存在、频率是否稳定：

```bash
rostopic list | grep -E "odom|imu|PowerVoltage|current_data|cmd_vel"
rostopic hz /odom
rostopic hz /imu
rostopic hz /current_data
rostopic echo -n 1 /current_data
```

检查 CSV 是否生成、字段是否完整：

```bash
ls -lh /home/wheeltec/R550PLUS_data_collect/log/
head -n 2 /home/wheeltec/R550PLUS_data_collect/log/normal_001.csv
tail -n 5 /home/wheeltec/R550PLUS_data_collect/log/normal_001.csv
```

重点观察 CSV 中的 `record_rate`、`odom_age`、`imu_age`、`current_age`、`cmd_age`。如果 `cmd_age` 很大，说明采集时没有稳定发布 `/cmd_vel`；如果 `current_age` 持续很大，说明电流板串口或 `/current_data` 频率异常。

---

## 定速巡航使用方法

1. 浏览器打开 `http://<robot-ip>:8000`
2. 连接 Rosbridge
3. 在"定速巡航"面板设置目标速度（Vx/Vy/Wz滑块）
4. 点击"启动巡航"
5. 机器人自动保持设定速度
6. 点击"停止巡航"退出

**巡航控制逻辑：**
- `/web/cruise_enable` (Bool) - 控制巡航开启/关闭
- `/web/cruise_cmd` (Twist) - 设置目标速度
- 摇杆控制不会退出巡航模式
- 巡航只能通过"停止巡航"按钮或 E-stop 退出

**rostopic 命令示例：**
```bash
# 匀速 0.5m/s 前进
rostopic pub /web/cruise_cmd geometry_msgs/Twist '{linear: {x: 0.5}}'
rostopic pub /web/cruise_enable std_msgs/Bool '{data: true}'

# 停止巡航
rostopic pub /web/cruise_enable std_msgs/Bool '{data: false}'
```

---

## 数据集构建流程

采集完成后，推荐使用 `build_fault_dataset.py` 将多个 run 的 CSV 转为深度学习样本。该脚本会按 `run_id` 划分训练集和测试集，避免重叠滑窗造成数据泄漏。

```bash
python ~/ml_robot_ws/src/turn_on_wheeltec_robot/scripts/build_fault_dataset.py \
  --input_dir /home/wheeltec/R550PLUS_data_collect/log \
  --output_dir /home/wheeltec/R550PLUS_dataset/v1 \
  --window_size 50 \
  --step_size 10 \
  --scaler standard
```

输出文件：

```bash
X_train.npy
y_train.npy
X_test.npy
y_test.npy
meta_train.csv
meta_test.csv
scaler.pkl
feature_config.json
```

默认模型输入特征为：

```text
cmd_vx, cmd_vy, cmd_wz,
odom_vx, odom_vy, odom_wz,
imu_ax, imu_ay, imu_az,
imu_gx, imu_gy, imu_gz,
voltage,
current0, current1, current2
```

如果 CSV 中存在有效的 `wheel_speed0~2`，脚本会自动加入轮速特征。输出的 `X_train` 形状为 `[num_samples, num_channels, window_size]`，适合直接输入 PyTorch `Conv1d` 模型。

---

## 数据说明

### 采样率

- **ROS 话题频率：20Hz**（当前由底盘控制器软件上报周期决定）
- **IMU 内部采样：100Hz**（固件滤波后对外发布 20Hz）
- 当前推荐 50 点滑动窗口 = **2.5秒**数据 @ 20Hz
- 当前推荐 10 点步长 = **0.5秒**

### 时间同步

数据采集时独立记录各话题时间戳和 age：
- `/odom`, `/imu`, `/PowerVoltage`：20Hz（固件同步发布）
- `/current_data`：由电流板串口上报，当前固件理论可到 100Hz，实际以 `rostopic hz /current_data` 为准
- `/cmd_vel`：必须记录，用于比较期望运动与实际运动反馈

### CSV 字段

```
timestamp,run_id,sample_id,fault_label,fault_name,fault_mode,motion_mode,
cmd_vx,cmd_vy,cmd_wz,odom_vx,odom_vy,odom_wz,
wheel_speed0,wheel_speed1,wheel_speed2,
imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,
voltage,current_seq,current0,current1,current2,
odom_time,imu_time,voltage_time,current_time,cmd_time,
odom_age,imu_age,voltage_age,current_age,cmd_age,record_rate,current_valid
```

| 字段 | 说明 | 单位 | 来源 |
|------|------|------|------|
| timestamp | 记录基准时间戳 | 秒 | /odom 时间戳 |
| run_id | 采集批次 ID | - | launch 参数 |
| sample_id | 当前 CSV 内样本序号 | - | data_collector |
| fault_label/fault_name | 故障标签 | - | launch 参数 |
| fault_mode | 软件故障注入模式 | - | launch 参数 |
| motion_mode | 标准运动工况 | - | launch 参数 |
| cmd_vx, cmd_vy, cmd_wz | 期望速度 | m/s, rad/s | /cmd_vel |
| odom_vx, odom_vy, odom_wz | 实际运动反馈 | m/s, rad/s | /odom |
| wheel_speed0~2 | 三轮轮速，当前预留 | m/s | 可选话题 |
| imu_ax~az | 加速度 | m/s² | /imu |
| imu_gx~gz | 角速度 | rad/s | /imu |
| voltage | 电池电压 | V | /PowerVoltage |
| current_seq | 电流帧序号，当前由 ROS 端递增 | - | /current_data |
| current0~2 | 三通道电流 | A | /current_data |
| odom_time | /odom 原始时间戳 | 秒 | 底盘串口 |
| imu_time | /imu 原始时间戳 | 秒 | 底盘串口 |
| voltage_time | /PowerVoltage 原始时间戳 | 秒 | 底盘串口 |
| current_time | /current_data 原始时间戳 | 秒 | 电流传感器 |
| cmd_time | /cmd_vel 接收时间戳 | 秒 | ROS |
| odom_age~cmd_age | 写入 CSV 时各话题数据年龄 | 秒 | data_collector |
| record_rate | CSV 实际写入频率 | Hz | data_collector |
| current_valid | 当前行是否已有有效电流数据 | - | data_collector |

**注意**：`timestamp`、`run_id`、`sample_id`、`fault_name`、各类 `*_time` 和 `*_age` 字段只用于记录和质量检查，不作为第一版模型输入。

### 默认输出目录

- `/home/wheeltec/R550PLUS_data_collect/log/`

### 归一化策略

`build_fault_dataset.py` 支持 `standard` 和 `minmax`。归一化参数只用训练 run 计算，再应用到测试 run，避免测试集泄漏。

---

## 参数配置

### data_collector.launch

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `fault_label` | `0` | 故障标签 |
| `fault_name` | 空 | 空时由 `fault_label` 自动映射 |
| `fault_mode` | `0` | 软件故障注入模式记录 |
| `run_id` | 空 | 空时自动生成 |
| `motion_mode` | `straight_0.5ms` | 运动工况 |
| `output_dir` | `/home/wheeltec/R550PLUS_data_collect/log` | CSV 输出目录 |
| `current_port` | `/dev/ttyUSB0` | 电流板串口 |
| `current_baud` | `115200` | 电流板波特率 |
| `wheel_speed_topic` | 空 | 可选轮速话题 |

### data_collector.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~fault_label` | `0` | 故障标签 |
| `~fault_name` | 空 | 故障名称 |
| `~fault_mode` | `0` | 故障注入模式 |
| `~run_id` | 自动生成 | 采集批次 ID |
| `~motion_mode` | `straight_0.5ms` | 运动工况 |
| `~output_dir` | `/home/wheeltec/R550PLUS_data_collect/log` | 输出目录 |
| `~cmd_topic` | `/cmd_vel` | 控制指令话题 |
| `~wheel_speed_topic` | 空 | 可选轮速话题 |

### cmd_vel_web_adapter.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~max_linear_x` | `0.4` | 最大前进速度 |
| `~max_linear_y` | `0.4` | 最大横移速度 |
| `~max_angular_z` | `1.0` | 最大角速度 |
| `~cmd_timeout` | `0.5` | 命令超时（秒） |
| `~heartbeat_timeout` | `1.0` | 心跳超时（秒） |
| `~response_exponent` | `0.70` | 响应曲线指数 |
| `~min_linear_ratio` | `0.20` | 最小前进比例 |

### web_control.launch

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `start_base` | `true` | 是否启动底盘节点 |
| `start_current_reader` | `true` | 是否启动电流采集节点 |
| `start_data_collector` | `true` | 是否启动数据采集节点 |
| `rosbridge_port` | `9090` | rosbridge 端口 |
| `web_port` | `8000` | Web 页面端口 |
| `publish_rate` | `40.0` | 控制发布频率（Hz） |
| `cmd_timeout` | `0.5` | 命令超时（秒） |
| `max_linear_x` | `1.50` | 最大前进速度 |
| `max_linear_y` | `1.00` | 最大横移速度 |
| `max_angular_z` | `3.75` | 最大角速度 |

---

## 调试命令

```bash
# 查看话题
rostopic list | grep -E "odom|imu|PowerVoltage|current|cmd_vel|web"

# 查看话题频率（确认 20Hz）
rostopic hz /odom
rostopic hz /imu
rostopic hz /current_data

# 查看实时数据
rostopic echo /odom
rostopic echo /imu
rostopic echo /PowerVoltage
rostopic echo /current_data
rostopic echo /web/control_status
rostopic echo /web/cruise_status

# 发送巡航命令
rostopic pub /web/cruise_cmd geometry_msgs/Twist '{linear: {x: 0.5}}'
rostopic pub /web/cruise_enable std_msgs/Bool '{data: true}'

# 查看数据采集输出
ls -la /home/wheeltec/R550PLUS_data_collect/log/
```

---

## 常见问题

### 1. 电流数据为空
```bash
rostopic echo /current_data
ls -l /dev/ttyUSB0
```
检查串口 `/dev/ttyUSB0` 是否存在，波特率是否正确。

### 2. 数据采集频率低
当前底盘上报任务为 20Hz，控制闭环和编码器更新为 100Hz。后续可在固件中按 20Hz → 50Hz → 100Hz 逐步测试，但需要同时检查丢包率、校验错误、时间戳间隔、重复帧和 CPU 占用。

### 3. 定速巡航速度不准
PID 参数需根据地面情况调整。当前参数适合瓷砖地面。

### 4. 浏览器无法连接
- 确认 `web_control.launch` 已启动
- 确认 `9090` 端口可访问
- 确认页面中 WebSocket 地址是机器人实际 IP

### 5. 节点冲突
同时启动 data_collector.launch 和 web_control.launch 会导致节点冲突。使用分开启动方式。

---

## 文件清单

```
turn_on_wheeltec_robot/
├── src/
│   ├── wheeltec_robot.cpp          # 底盘控制节点源码
│   └── Quaternion_Solution.cpp     # IMU四元数解算
├── scripts/
│   ├── current_reader.py            # 电流读取脚本
│   ├── data_collector.py           # 数据采集脚本（/odom触发，最新值填充）
│   ├── build_fault_dataset.py      # 多run CSV转滑窗深度学习数据集
│   ├── cmd_vel_web_adapter.py      # Web控制适配器（PID巡航）
│   ├── preprocess_data.py          # 数据预处理
│   ├── create_sliding_windows.py   # 滑动窗口分割
│   ├── validate_dataset.py          # 数据集验证
│   └── web_dashboard_server.py      # Web页面服务
├── web/
│   └── index.html                  # Web控制页面（含巡航UI）
├── launch/
│   ├── turn_on_wheeltec_robot.launch  # 底盘启动
│   ├── data_collector.launch         # 数据采集启动
│   ├── web_control.launch           # Web控制启动
│   └── include/base_serial.launch   # 底盘串口配置
└── README.md
```

---

## 作者

杨鹏 - 燕山大学本科毕业设计



数据采集流程一览

┌─────────────────────────────────────────────────────────────┐
│  采集前准备                                                │
│  - 机器人开机                                              │
│  - 确认串口正常 /dev/ttyUSB0, /dev/ttyCH343USB0           │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  分开启动（三个终端，避免节点冲突）                       │
│  终端1: roslaunch turn_on_wheeltec_robot                   │
│              turn_on_wheeltec_robot.launch                  │
│  终端2: roslaunch turn_on_wheeltec_robot                   │
│              data_collector.launch fault_label:=X           │
│  终端3: roslaunch turn_on_wheeltec_robot                   │
│              web_control.launch start_base:=false \          │
│              start_current_reader:=false \                  │
│              start_data_collector:=false                   │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  控制机器人移动                                            │
│  浏览器打开 http://<robot-ip>:8000                         │
│  使用摇杆/定速巡航控制机器人匀速移动                       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  采集足够时间后 Ctrl+C 停止                               │
│  采集 10-15 分钟/种故障类型                               │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  预处理（机器人或本地）                                    │
│  python preprocess_data.py --data_dir ... --pattern '*.csv' │
│  python create_sliding_windows.py --data_path ...           │
│  python validate_dataset.py --data_dir ...                  │
└─────────────────────────────────────────────────────────────┘
