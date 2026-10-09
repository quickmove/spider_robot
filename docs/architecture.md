# 软件架构（M1~M3）

## 1. 设计原则

1. **硬件恒定，软件为变量**：8 舵机固定，所有能力扩展在软件层
2. **舵机写入权唯一**：只有 `motion` 任务能调用 `servo_set`，其余组件只发命令
3. **组件解耦**：每个组件可独立编译与单元测试，通过头文件接口交互
4. **参数同源**：几何/步态/时序参数唯一定义在 `tools/gait_sim/params.yaml`，固件编译期生成头文件
5. **拟真优先**：默认行为模仿昆虫（谨慎行走、快慢节奏、假死），非拟真行为需显式请求

## 2. 组件划分与依赖

```
                 ┌──────────────────────────────┐
   WebSocket ───▶│        web_ctrl               │
   串口命令  ───▶│  （协议解析 / 会话管理 / 页嵌） │
                 └──────────────┬───────────────┘
                                │ command_t（队列）
                                ▼
                 ┌──────────────────────────────┐
                 │            motion             │  唯一舵机写入者
                 │  任务 1~2ms tick / 斜坡 / 限位  │
                 └──┬────────┬────────┬─────────┘
                    │        │        │
        ┌───────────▼─┐ ┌────▼─────┐ ┌▼──────────┐
        │   posture   │ │   gait   │ │ 滑移原语   │
        │ 关键帧播放器 │ │ 步态引擎  │ │ nudge/lean│
        └───────┬─────┘ └────┬─────┘ └─────┬─────┘
                └────────────┼──────────────┘
                             ▼
                    ┌────────────────┐
                    │   kinematics   │  足端目标 → 髋/膝角
                    └────────┬───────┘
                             ▼
                    ┌────────────────┐
                    │     servo      │  角度→脉宽 / 标定 / NVS
                    └────────┬───────┘
                             ▼
                    ┌────────────────┐
                    │    pca9685     │  I2C 寄存器
                    └────────────────┘
```

依赖严格单向向下。`kinematics` 不引用 `gait`；`gait` 不引用 `motion`。

## 3. FreeRTOS 任务

| 任务 | 周期/触发 | 优先级 | 栈 | 职责 |
|---|---|---|---|---|
| `motion` | 1~2ms 周期 tick | 5（最高） | 4096 | 消费命令队列、跑步态/姿态状态机、调用 servo 写入 |
| `wifi` | IDF 默认 | 5 | 4096 | 由 esp_wifi 管理 |
| `web` | 事件驱动 | 4 | 8192 | httpd + WebSocket 收发，会话心跳 |
| `telemetry` | 100ms | 2 | 2048 | 状态回传、电池/温度监测 |
| `cli` | 行驱动 | 3 | 4096 | 串口命令（调试用，与 Web 同一命令集） |

**关键约束**：`motion` 独占 `servo_set`。任何其他路径（含 httpd 回调、CLI）只能 `motion_enqueue()`。

## 4. 命令模型

### 4.1 命令队列

```
typedef enum {
    CMD_SET_POSTURE,      // 切换姿态/关键帧序列
    CMD_GAIT_START,       // 启动步态（爬行/trot）
    CMD_GAIT_STOP,
    CMD_SET_SPEED,        // 0~100%
    CMD_SET_HEADING,      // 目标航向（度）
    CMD_NUDGE,            // 滑移微调 (dx, dy) mm
    CMD_LEAN,             // 姿态偏移 (roll, pitch) 度
    CMD_ESTOP,            // 急停（最高优先级，立即生效）
    CMD_CALIBRATE,        // 进入标定模式
    CMD_CAL_SAVE,
    CMD_ESTOP_CLEAR,
} cmd_type_t;
```

队列为环形缓冲，长度 16。`CMD_ESTOP` 走单独标志位（`volatile bool`），入队之外还直接置位，`motion` tick 每次检查——保证不因队列满而失效。

### 4.2 速度斜坡

所有角度变化经 `slew_limit()`：单 tick 内角度变化不超过 `max_deg_per_tick`。

```
max_deg_per_tick = servo_rated_speed_deg_per_sec * tick_period
```

996R @6V 空载 428°/s，但带载约 1/2，且**舵机层设全局角速度上限**（默认 180°/s，标定时可单独放宽单舵机）。

### 4.3 优先级与抢占

- 急停 > 姿态/关键帧 > 步态 > 微调
- 步态运行中收到姿态命令：步态停止 → 缓动过渡到目标姿态（不瞬跳）
- 任何时候收到急停：所有关节输出当前角、停止步态、清空队列

## 5. 坐标系与角度约定

### 5.1 机身坐标系

X 前、Y 左、Z 上，单位 mm / 度。定义见 docs/hardware.md §1。

### 5.2 关节角

| 量 | 符号 | 范围 | 说明 |
|---|---|---|---|
| 髋角 | `hip[i]` | -60°~+60° | 0 = 外八中立位（腿平面方位角 = 名义方位角）；+ = 腿向前摆 |
| 膝角 | `knee[i]` | 20°~75° | β = 小腿与水平面夹角；小 β = 屈膝抬高；软限位见 §5.3 |
| 机身高度 | `h` | 90~150mm | 由四膝决定 |

### 5.3 软限位（servo 层强制）

| 关节 | 软限位 | 依据 |
|---|---|---|
| 髋 | ±60° | 舵机机械范围 + 支架干涉边界 |
| 膝（β） | 20°~75° | 站立高度 150mm(β≈20) ~ 90mm(β≈75)；β>75° 腿几乎伸直，髋摆干涉 |
| 全局角速度 | 180°/s | 996R 带载速 + 齿隙保护 |

软限位在 `servo_set_target()` 内 clamp，**任何路径都无法绕过**。

## 6. 2DOF 运动学模型

### 6.1 单腿几何

足端在腿平面内为极坐标 `(r, θ)`，以髋轴心为原点；θ 为足端方位角相对腿平面水平轴的夹角（相对中位向上为正）。

```
大腿 L1（α 固定，支架决定）
小腿 L2（β 随膝角变化）

膝角 β 给定 → 腿长：
  r = L1 + L2·cos(β_swing)      // β_swing = 膝相对大腿的弯曲量
足端垂直高度（相对髋）：
  z = -L1·sin(α) - L2·sin(β)
```

**实际实现采用简化模型**（gait_sim 与固件一致）：

- 足端目标用 **(方位角 ψ, 半径 r)** 描述，ψ 由髋角直接给出
- 高度由膝角 β 直接给出：`h = L1·sin(α) + L2·sin(β)`
- 髋角 = ψ（腿平面旋转）+ 相对中位的扫摆量
- 膝角 = acos(clamp((r - L1)/L2, -1, 1)) 反解

**坐标一致性**：ψ=0 表示足端在腿平面水平轴正方向（即腿完全向外水平伸出）。这不是常见姿态，仅作数学基准。真实巡航姿态 ψ 有偏移以使站立高度符合 §5.2。

### 6.2 逆运动学（`kinematics`）

输入：足端目标 `(ψ, r)`（腿平面极坐标）或 `(ψ, h)`（方位角 + 高度）
输出：`{hip[i], knee[i]}`

```
hip[i]  = clamp(ψ - ψ_nominal[i], HIP_MIN, HIP_MAX)
knee[i] = acos(clamp((r - L1) / L2, -1, 1))   →  转成 β 后 clamp
```

正解（用于 gait_sim 与仿真校验）：

```
r  = L1 + L2·cos(β_bend)
h  = L1·sin(α) + L2·sin(β)
```

### 6.3 变高的运动学耦合（重要）

膝角 β 转动时，足端**同时**在水平面内移动。耦合比：

```
高度变化 : 足端水平位移 = 1 : cot(β)
```

| β | 耦合比 | 含义 |
|---|---|---|
| 90°（腿伸直） | 1 : 0 | 膝动只改高度，但此位 dH/dβ ≈ 0（不灵敏） |
| 75° | 1 : 0.27 | 高度灵敏但仍易拖脚 |
| **45°（巡航）** | **1 : 1** | 平衡点：升降与平移代价相当 |
| 30°（深蹲） | 1 : 1.73 | 高度极灵敏但腿软 |

**设计决策**：巡航 β=45°，绝不设计在伸直位。理由：

1. 伸直位 dH/dβ ≈ 0，变高效率极低且必须拖脚几十毫米
2. 屈膝位舵机工作在中段，扭矩与回差最好
3. 巡航屈膝可让齿隙单向贴合，减少残差

α 与 β 的最终取值由 `tools/gait_sim` 扫参确定（见 docs/gait_design.md §2）。

## 7. 组件接口（概要）

```c
// pca9685
esp_err_t pca9685_init(i2c_port_t port, uint8_t addr);
esp_err_t pca9685_set_pulse_us(uint8_t ch, uint16_t us);
esp_err_t pca9685_set_freq(uint16_t freq_hz);   // 50Hz
esp_err_t pca9685_sleep(bool on);

// servo
esp_err_t servo_init(void);
esp_err_t servo_set_target(uint8_t leg, uint8_t joint, float deg);  // 含斜坡与限位
esp_err_t servo_apply_all(void);                 // 由 motion 独占调用
esp_err_t servo_cal_set(uint8_t idx, const servo_cal_t *cal);
esp_err_t servo_cal_save(void);
esp_err_t servo_cal_load(void);
esp_err_t servo_detect_servo(uint8_t ch);       // 标定模式用

// kinematics
void leg_ik(float psi_deg, float r_mm, leg_ik_t *out);

// gait
typedef struct {
    gait_type_t type;       // CRAWL | TROT
    float stride_mm;
    float period_s;
    float lift_mm;
    float speed_pct;
    float heading_deg;
} gait_params_t;
esp_err_t gait_init(void);
void gait_update(float dt, float *body_vx, float *body_vy, float *body_yawrate, leg_cmd_t *cmds);

// posture
esp_err_t posture_play(const char *name);
esp_err_t posture_keyframe_push(const kf_t *frames, size_t count);

// motion（应用层）
esp_err_t motion_init(void);
esp_err_t motion_enqueue(const motion_cmd_t *cmd);
void motion_estop(void);
void motion_estop_clear(void);
```

## 8. NVS 布局

命名空间 `spider`：

| 键 | 类型 | 内容 |
|---|---|---|
| `cal` | blob | 8 路舵机标定数组：`{min_us, max_us, trim_us, reverse, neutral_us}` |
| `geom` | blob | 逐机几何实测：`{L1, L2, α, β_cruise}` |
| `gait` | blob | 上次步态参数（爬行与 trot 各一套） |
| `wifi` | blob | STA 配置（AP 模式不需要） |

标定数据 Web 保存后立即生效（NVS 写入为异步，不阻塞 motion 任务）。

## 9. Web 架构

```
esp_netif → esp_wifi(AP) → esp_http_server
                                 ├── GET /           → 内嵌单文件控制页（gzip，EMBED_FILES）
                                 ├── WS   /ws        → 控制协议（见 docs/protocol.md）
                                 ├── GET  /api/cal   → 读取标定
                                 └── POST /api/cal   → 保存标定
```

控制页零外部依赖：单 HTML 文件内联 CSS/JS，离线可用。内容通过 `EMBED_FILES` 编入固件分区，不占用文件系统。

### 会话与安全

- 每次连接分配 session，上报时间戳
- **心跳超时 500ms 无指令 → 自动急停**（防止手机息屏后机器人继续走）
- AP 名称 `SpiderRobot-XXXX`（MAC 后 4 位），无密码或固定密码（Kconfig 可配）
- 急停按钮在 UI 上常驻、红色、触达友好

## 10. 二期扩展位（不在一期实现）

| 扩展 | 接口预留 |
|---|---|
| BLE 遥控 | `web_ctrl` 改为协议无关层，命令集共用 |
| 雷达（UART/ I2C） | 新组件 `sensor_lidar`，经命令队列下发 `CMD_GAIT_*` |
| 相机 / 视觉 | ESP32-S3 或 RV1106 协处理器，经 UART 发高层指令（`motion` 不动） |
| IMU（翻车检测） | 新组件 `sensor_imu`，仅触发行为包，不参与控制环 |
| 弹性肌腱弹跳 | `servo` 层增加张力控制，实验性 |

**所有扩展都通过命令队列进入，不改动 `motion` 内部逻辑**——这是组件划分的第一原则。

## 11. 坐标系一致性校验（跨工具）

以下三处必须用同一套定义，任何改动需同步：

1. 固件 `components/kinematics`
2. `tools/gait_sim/ik.py`
3. `tools/blender/build_rig.py`（骨架生成）

由 `tools/gait_sim/params.yaml` 作为单一数据源；Blender 脚本与固件头文件均从此生成。交叉校验方法：用同一组随机足端目标，三处解出的 8 个关节角必须逐位一致（在 params.yaml 精度内）。