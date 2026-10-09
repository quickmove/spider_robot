# 控制协议（M3）

WebSocket 与串口 CLI 共用同一套命令集与字段定义。协议为 JSON 文本，便于浏览器与串口终端双向调试。

## 1. 传输层

| 通道 | 端点 | 说明 |
|---|---|---|
| WebSocket | `ws://<ip>/ws` | 主控制通道，JSON 帧 |
| HTTP GET | `/` | 控制页（内嵌固件） |
| HTTP GET | `/api/cal` | 读取 8 路标定（JSON） |
| HTTP POST | `/api/cal` | 保存标定（body 为 JSON） |
| HTTP GET | `/api/params` | 读取当前步态/几何参数 |
| 串口 CLI | 115200 8N1 | 行分隔命令，见 §5 |

## 2. 命令方向

```
页面/终端  ──── cmd ────▶  机器人
页面/终端  ◀─── ack ────  机器人   （ack 仅对关键命令）
机器人      ◀─── st ────  页面/终端 （状态回传，10Hz）
```

### 2.1 控制输入 `cmd`

| 字段 | 类型 | 单位/范围 | 说明 |
|---|---|---|---|
| `t` | string | — | 命令类型，见 §3 |
| `id` | int | 0~65535 | 可选；存在时机器人回 `ack{id}` |
| `seq` | int | 递增 | 可选；用于丢包检测 |

### 2.2 摇杆连续量 `cmd:drive`

页面以 10~20Hz 上报，机器人内部按心跳处理。

```json
{"t":"drive","vx":-0.35,"vy":0.8,"yaw":0.0}
```

| 字段 | 含义 | 说明 |
|---|---|---|
| `vx` | 前后意图 | -1 后退，+1 前进 |
| `vy` | 左右意图 | 映射到侧移宏（见 docs/gait_design.md §5），不是真正横移 |
| `yaw` | 转向意图 | -1 右，+1 左 |

**心跳规则**：连续 500ms 未收到任何 `cmd` → **自动停止行走、保持当前姿态**（不锁存急停标志）。

为什么不锁存急停：Web 断连是常态（手机息屏、切后台、走出范围）。
若锁存则机器人进入 `ESTOP` 态、需手动 `estop_clear` 才能恢复，等于远程失控。
真正的急停只由显式 `estop` 命令产生。页面另有三重主动停止：
摇杆松开即刻发 `drive(0,0,0)`；`visibilitychange` 时发零值 + `stop`；
WebSocket 关闭时服务端也清零运动意图（不等待 500ms）。

### 2.3 状态回传 `st`

10Hz 推送。

```json
{
  "t":"st",
  "mode":"WALK",
  "posture":"STAND",
  "gait":"TROT",
  "speed":42,
  "heading":-7,
  "legs":[[12,-31],[14,-29],[-11,-33],[-13,-30]],
  "temp":42,
  "uptime":3600
}
```

| 字段 | 说明 |
|---|---|
| `mode` | `IDLE` / `WALK` / `POSTURE` / `KEYFRAME` / `CALIBRATE` / `ESTOP` |
| `legs` | `[[hip, knee], ...]` × 4，顺序 FL, FR, RL, RR，角度单位度 |
| `temp` | ESP32 结温，°C |
| `estop` | 存在时为 `true` |

## 3. 命令类型全集

### 3.1 运动

| `t` | 参数 | 行为 |
|---|---|---|
| `drive` | `vx,vy,yaw` | 连续行走意图（见 §2.2） |
| `gait` | `type` | 切换步态：`"crawl"` / `"trot"` |
| `speed` | `pct` (0~100) | 速度百分比 |
| `stop` | — | 停止行走，保持当前姿态 |
| `nudge` | `dx,dy` (mm, ≤50) | 滑移微调（棘轮搓动），见 docs/gait_design.md §6 |
| `lean` | `roll,pitch` (度, ±10) | 差动膝姿态偏移 |
| `strafe_macro` | `dx` (mm) | 侧移宏：转向→直行→转回，内部执行，进度经 `st.mode` 反馈 |

### 3.2 姿态与动作

| `t` | 参数 | 行为 |
|---|---|---|
| `pose` | `name` | `stand` / `sit` / `rest` / `spawn`（翻车姿态） |
| `height` | `mm` | 原地变高 90~150（摆动相内逐腿执行，零滑移） |
| `play` | `name` | 播放动作序列（Blender 导入或内置拟真行为） |
| `play_list` | — | 请求可用序列名（回 `ack{list:[...]}`） |
| `keyframe_upload` | 见 §4 | 上传自定义关键帧 |

### 3.3 标定

| `t` | 参数 | 行为 |
|---|---|---|
| `cal_enter` | — | 进入标定模式（步态停止，舵机可单控） |
| `cal_exit` | — | 退出标定模式 |
| `cal_scan` | — | 逐路扫动测试（低速小幅，检测通道接线） |
| `cal_set` | `ch,min,max,neutral,trim,reverse` | 设置单路标定（见 §6） |
| `cal_pulse` | `ch,us` | 标定模式下直写单路脉宽，供 Web 滑条实时预览 |
| `cal_save` | — | 写 NVS |
| `cal_load` | — | 从 NVS 读 |

### 3.4 安全与系统

| `t` | 参数 | 行为 |
|---|---|---|
| `estop` | — | 急停，立即生效，置 `estop` 标志 |
| `estop_clear` | — | 清除急停（需先回 `stand` 姿态） |
| `ping` | — | 回 `ack{ping:...}`，测延迟 |
| `reboot` | — | 重启 |
| `set_params` | 见 §7 | 运行时改步态/几何参数（不落 NVS） |
| `save_params` | — | 参数落 NVS |
| `advanced` | `on` | 开关高级模式（放开防翻硬限位，Web 页默认关闭） |

## 4. 关键帧上传

用于 Blender 导出的动作序列。

```json
{
  "t":"keyframe_upload",
  "name":"wave",
  "period_ms":0,
  "loop":true,
  "ease":"smoothstep",
  "frames":[
    {"t":0,   "legs":[[10,-30],[10,-30],[10,-30],[10,-30]]},
    {"t":250, "legs":[[40,-20],[10,-30],[10,-30],[10,-30]]}
  ]
}
```

| 字段 | 说明 |
|---|---|
| `t` | 相对起点的毫秒 |
| `legs` | 8 个关节角，顺序 FL.h, FL.k, FR.h, FR.k, RL.h, RL.k, RR.h, RR.k |
| `ease` | 插值：`linear` / `smoothstep` / `ease_in_out` |
| `loop` | 循环播放（`period_ms=0` 表示直到收到新命令） |

限制（实现值，与 `spider_kf_parse.h` 一致，由 `protocol_check.py` 校验）：

| 限制 | 值 | 出处 |
|---|---|---|
| 单序列帧数 | ≤ 512 | `KF_MAX_FRAMES` |
| 单条报文 | ≤ 4KB（4096 字节） | `KF_MAX_UPLOAD_BYTES` |
| 序列名 | ≤ 23 字符 | `kf_seq_t.name[24]` |
| 同时驻留序列 | **2 条** | `KF_MAX_SEQS` |
| 每帧 `legs` | 必须恰好 8 个数 | `POSTURE_JOINTS` |

超限回 `ack{error:"too_large"}`，其余非法字段回 `bad_field`。序列存于 RAM，
不落 NVS（刷机丢失，可重新上传）。

**为何报文上限是 4KB 而非 32KB**：不是收发缓冲不够，而是 cJSON 的内存代价。
每帧 JSON 展开后约 19 个节点（帧对象 + t + legs 数组 + 8 个子数组 + 8 个数字），
每节点约 64 字节。32KB 报文若真填满 512 帧，cJSON 树需要约 160KB 堆——超过
ESP32 可用余量，必然分配失败。4KB 约合 60 帧手势序列，解析峰值约 70KB，可用。

同理 `KF_MAX_SEQS` 从 4 降到 2：单条 `kf_seq_t` 内联 512 帧 × 36B ≈ 18.5KB，
4 条即 74KB 常驻堆。调试阶段一次只需跑一个动作，不值得为此挤占运行时余量。
需要更多序列时应改为按实际帧数动态分配，而非调大此值。

## 5. 串口 CLI

```
help                        列出所有命令
ping                        连通性与延迟
status                      模式、姿态、关节角、支撑裕度

pose stand|sit|rest|spawn   切换姿态
height <mm>                 原地变高 90~150mm（摆动相执行，不拖脚）
gait crawl|trot             选择步态并开始行走
speed <0-100>               速度百分比
heading <deg>               目标航向
walk <vx> <yaw>             连续行走（各 -1~1）
walk_stop                   停止行走
strafe <mm>                 侧移宏（转向-直行-回转）
nudge <dx> <dy>             滑移微调（限幅 50mm）
lean <roll> <pitch>         差动膝姿态（限幅 ±10°）
estop                       急停
estop_clear                 解除急停

cal show                    查看 8 路标定
cal enter                   进入标定模式
cal exit                    退出标定模式
cal scan                    逐路扫动，检查通道对应与极性
cal set <ch> <min> <max> <neu> <trim> <rev>
cal save                    保存到 NVS
cal load                    从 NVS 载入
cal reset                   恢复出厂标定
servo <ch> <us>             标定模式下直写脉宽

leg show                    8 路标定与当前脉宽
leg sweep <ch> <from> <to> [fast|normal|slow | step_us [dwell_ms]]
leg step <ch> <+1|-1> [step_us]   单步逼近（找机械极限首选）
leg hold <ch> <us>          保持指定脉宽
leg release                 8 路回中位

play <name>                 【预留】播放关键帧序列（仅 Web 已实现）
play_list                   【预留】列出可用序列（仅 Web 已实现）
params show                 【预留】参数查询（尚未实现）
params set <key> <val>      【预留】运行时改参数（尚未实现）
reboot                      【预留】重启（仅 Web 已实现）
```

CLI 输出为人类可读文本（`OK ...` / `ERR ...`），非 JSON，便于串口终端直接看。

### 5.1 §5 与 §3 的关系

**§5 是串口 CLI 命令集，§3 是 Web 协议命令集，两者不是同一套。**

- 同名命令（`cal enter`、`pose`、`gait` 等）参数与语义一致
- `keyframe_upload`、`set_params` 等**仅存在于 Web**，串口侧无对应命令
- 标 `【预留】` 的条目串口侧尚未实现，敲下去会回 `ERR 未知命令`；
  括号内标明 Web 侧的实现状态

标 `【预留】` 是为了让检查能区分「文档漏记」与「有意留白」：真实实现但未写进本节的命令会被
`protocol_check.py` 拦下；而预留行若混入了已实现命令，同样会被拦。

`leg` 组仅在 `CONFIG_SPIDER_BENCH_MODE=y` 的固件上有意义（见 docs/calibration.md §0.5）。
其区间参数必须落在该路已标定的 `[min_us, max_us]` 内，越界会被拒绝——因为出厂
默认的 1000~2000µs 正是 MG996R 的贴死区，必须先 `cal set` 收窄。

## 6. 标定数据结构

```json
{
  "ch": 0,
  "min": 1000,
  "max": 2000,
  "trim": 0,
  "reverse": false,
  "neutral": 1500
}
```

| 字段 | 单位 | 说明 |
|---|---|---|
| `min` / `max` | µs | 可用脉宽端点，留 5~10µs 余量，**严禁贴死** |
| `trim` | µs | 中位脉宽微调 |
| `reverse` | bool | 方向反转 |
| `neutral` | µs | 该舵机的机械中位（与标定后的几何中位对应） |

角度→脉宽映射：

```
pulse = neutral + (deg / span) * (max - min) + trim
span  = ±90（标定范围，对应 996R 常用区间）
reverse → pulse = 2*neutral - pulse
```

## 7. 参数集（与 params.yaml 同源）

```json
{
  "t":"set_params",
  "geom":{"L1":100.0,"L2":110.0,"alpha_deg":30.0,"beta_cruise_deg":39.5},
  "gait":{"stride_mm":60,"period_s":1.7,"lift_mm":30},
  "trot":{"stride_mm":68,"period_s":1.8,"lift_mm":35},
  "limits":{"hip_deg":60,"beta_min":21.3,"beta_max":65.4,"max_rate_deg_s":180}
}
```

字段与 `tools/gait_sim/params.yaml` 一一对应。固件编译期由该文件生成 `spider_params.h`，运行时 `set_params` 只改 RAM 副本，`save_params` 才落 NVS。

## 8. 错误与确认

| 响应 | 格式 |
|---|---|
| 确认 | `{"t":"ack","id":N,"ok":true}` |
| 拒绝 | `{"t":"ack","id":N,"ok":false,"error":"..." }` |
| 异步错误 | `{"t":"err","msg":"..."}` |

常见 `error` 取值：`bad_json` / `bad_field` / `mode_denied` / `out_of_range` / `too_large` / `cal_not_active` / `advanced_required`。

## 9. 安全不变量（固件强制，协议无法绕过）

1. **急停永远有效**：`estop` 走独立标志位，队列满也不失效；且立即拉高 PCA9685 的
   OE 引脚做**硬件级**全局关断（一次 GPIO 写，不经 I2C，I2C 挂死时依然有效）。
   OE 外接 10kΩ 上拉，故 ESP32 复位/崩溃/卡在 bootloader 时舵机同样瘫软
2. **软限位永远生效**：servo 层 clamp，任何路径不可绕。限位为**编译期常量**，
   协议层不可写（`set_params` 收到 `limits` 会回 `advanced_required`）
3. **心跳超时必停**：500ms 无指令停止行走并保持姿态；WebSocket 断开时立即清零意图
4. **标定模式互斥**：进入 `CALIBRATE` 时步态强制停止，单舵机输出经标定参数映射
5. **防翻约束默认锁死**：`advanced:off` 时 `lean`/转向限幅、高速步态受限（docs/gait_design.md §7）

这五条是硬约束，Web 页或串口发任何指令都不能突破。