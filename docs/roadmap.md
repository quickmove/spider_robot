# 里程碑与验收标准

每个里程碑独立编译验证后再进入下一个。工具链与固件同源参数，任何参数改动需在 `tools/gait_sim` 与固件间保持一致（docs/architecture.md §11）。

## M0 环境与骨架

| 项 | 内容 |
|---|---|
| 交付 | ESP-IDF v5.x 环境；`spider_robot` 项目骨架（顶层 CMakeLists、main/、sdkconfig.defaults） |
| 动作 | 安装 VSCode + ESP-IDF 扩展（或官方 install.sh）；`idf.py set-target esp32`；`idf.py build` |
| 验收 | 空项目零错误编译通过；`idf.py -p <port> flash monitor` 有启动日志 |
| 依赖 | 无 |

注意：macOS 下 USB 串口设备名形如 `/dev/cu.usbserial-*`。

## M0.5 硬件规格文档（已完成）

| 项 | 内容 |
|---|---|
| 交付 | docs/hardware.md、docs/architecture.md、docs/gait_design.md、docs/protocol.md、docs/calibration.md、docs/insect_behavior.md、README.md、tools/gait_sim/params.yaml |
| 验收 | 关节轴向、尺寸包络、重量预算、扭矩校核、供电接线、3D 打印件清单、BOM、通道映射、关节软限位、坐标系约定全部明确且互不矛盾 |

## M1 舵机硬件层 ✅ 代码完成，待实机验证

| 项 | 内容 |
|---|---|
| 交付 | `components/pca9685/`、`components/servo/` |
| pca9685 | IDF5 新版 `i2c_master` API，寄存器级（MODE1/预分频/通道 PWM/复位/全通道关），50Hz，12bit 4096 级，含探测自检 |
| pca9685_math | 纯算术分离（分频计算、脉宽↔计数 Q16 定点换算），可主机端测试 |
| servo | 8 路注册表、角度↔脉宽映射（端点锚定）、996R 出厂标定、软限位（髋 ±60°、β∈[21.3,65.4]）、角速度上限（默认 180°/s）、NVS 标定读写与完整性校验 |
| servo_logic | 纯逻辑分离（标定合法性、软限位、角速度斜坡、端点硬限幅），可主机端测试 |
| CLI | `cal show/enter/exit/scan/set/save/load/reset`、`servo`、`leg`（台架）、`pose`、`height`、`status` |
| 验证 | 主机端断言全通过（pca9685 18 项 + servo 11 项）；编译零警告零错误 |
| 待实机 | ① `cal scan` 通道号与 FL/FR/RL/RR 对应正确<br>② 极性正确（无反折）<br>③ 单舵机端点不顶死<br>④ 标定保存后重启仍生效<br>⑤ 实测软限位：设 70° 髋角被 clamp 到 60° |

实现要点：
- IDF 5.x 用 `driver/i2c_master.h`（新 API），`i2c_master_transmit_to` 在 5.4 已移除，SWRESET 改用 `i2c_master_transmit`
- `esp_vfs_dev_uart.h` 在 IDF 5.4 已移除，CLI 走 `fgets(stdin)`
- 角度↔脉宽以**标定端点为锚**：逻辑区间线性映射到 [min_us, max_us]，保证「软限住」与「物理端点」一致
- prescale 为 8bit，可达频率下限约 23.84Hz（`pca9685_math_prescale` 会拒绝越界频率）

## M2 运动层 ✅ 代码完成，待实机行走验证

| 项 | 内容 |
|---|---|
| 交付 | `components/kinematics/`、`components/gait/`、`components/posture/`、`components/motion/`、`components/cli/` |
| kinematics | 2DOF 腿逆解（高度↔膝角、足端半径、步幅↔髋摆角、变高耦合比），纯计算，15 项断言 |
| gait | 爬行 4 拍 + Trot 2 拍；支撑足世界坐标反解 → 模型层零滑移；精确差速步幅实现转向；reposture 实现原地零滑移变高；支撑裕度（凸包）指标。38 项断言 |
| posture | 内置姿态 REST/STAND/SIT/SPAWN + 关键帧播放器（linear/smoothstep/ease_in_out，循环，RAM 池按需分配） |
| motion | 2ms tick 任务、环形命令队列、急停独立标志位、心跳超时、侧移宏三段机、差动膝 lean、姿态/关键帧状态机 |
| cli | 应用层独立组件（避免 servo↔motion 循环依赖），22 条命令 |
| 验证 | 主机端 99 项断言 + gait_sim 双步态验收全通过；编译零警告 |
| 待实机 | ① 站立高度稳定 115~125mm 无抖动<br>② 爬行 20 周期直线偏离 <50mm<br>③ 转向能到 90°±8°<br>④ 急停 <50ms<br>⑤ 变高时目视脚垫几乎无滑移<br>⑥ Trot 直行易侧倾属预期（静态裕度恒为 0，见 docs/gait_design.md §3.5） |

### 静态稳定裕度的实测结论（重要）

四足支撑多边形最小边裕度 174mm，但**爬行是三足支撑**：

- 中心对称四足布局 → 任意三足支撑必有一条对角边过重心 → 裕度恰为 0
- 改腿长无效（±6mm 只带来 0.1mm）；唯一手段是打破前后对称
- 采用**前外八 36° / 后外八 48°** → 最差三足裕度 17.9mm（前进效率代价 4%）
- **Trot 静态裕度恒为 0**（支撑腿必为过重心的对角线对）→ 只靠动态稳定
- 这就是「默认爬行 + Trot 限速 70% + 转向限速 25°/s」三条防翻红线存在的唯一理由

详见 docs/gait_design.md §3.5。

实现注意：
- motion 是舵机写入的唯一所有者；其余路径只能 `motion_enqueue()`
- CLI 放在独立组件，否则 servo 与 motion 会循环依赖
- 新增组件后必须 `rm -rf build` 再编译——增量构建会静默跳过未识别的组件

## M3 Web 遥控

| 项 | 内容 |
|---|---|
| 交付 | `components/web_ctrl/`、内嵌控制页 |
| 内容 | WiFi AP（`SpiderRobot-XXXX`）、`esp_http_server` + WebSocket、单文件 HTML/JS 控制页（`EMBED_FILES`，gzip，零外部依赖） |
| 页面 | 虚拟摇杆、姿态按钮（站立/蹲坐/休息/翻车）、急停常驻按钮、标定模式（单舵机滑条 + 实时预览 + 保存）、动作序列按钮、状态回传（关节角/温度/模式） |
| 安全 | 心跳 500ms 超时自动急停；页面 `visibilitychange` 主动急停 |
| 验收 | ① 手机连热点打开页面，控制台无错误<br>② 摇杆连续行走，松开即停<br>③ 手机息屏后 500ms 内机器人停止<br>④ 标定滑条拖动实时生效，保存后重启保持<br>⑤ 急停按钮触达友好，单手可点<br>⑥ 页面断连后机器人不失控<br>⑦ 断电重启后热点自动起，控制页可访问 |

## M4 联调收尾

| 项 | 内容 |
|---|---|
| 交付 | 装配/标定文档补全、参数调优指南、README 完善、零警告编译 |
| 调参 | 用 gait_sim 扫参 → 实机验证 → 锁定 params.yaml → 固件同步 |
| 验收 | ① `idf.py build` 零警告<br>② docs/calibration.md 流程由未参与设计者可独立完成装配与标定<br>③ 机器人连续行走 10 分钟无异常（舵机不过热、脚垫不松、NVS 无异常写入）<br>④ 所有文档数值与固件实测值一致 |

## 工具链（与 M1~M3 并行）

| 工具 | 交付 | 状态 | 验收 |
|---|---|---|---|
| `tools/gait_sim/` | params.yaml、ik.py、gait.py、preview.py | ✅ 已完成 | 满足 docs/gait_design.md §9 全部验收项；能扫参并报告滑移指标 |
| `tools/blender/` | build_rig.py（参数化骨架）、export addon（→JSON）、kf_convert.py（JSON→C） | ⬜ **未开始** | 生成的骨架关节轴向与 hardware.md §1 一致；导出序列经固件播放姿态正确 |

依赖：`numpy`、`matplotlib`、`pyyaml`（pip）。Blender 侧用内置 bpy，无外部依赖。

> `tools/blender/` 目录**尚未创建**。在它之前，`keyframe_upload` 只能手工构造 JSON 上传；
> 协议层（`components/posture/src/kf_parse.c`）已就绪并有 61 项断言覆盖，只缺上游的导出工具。

## 二期候选（不在一期范围）

| 项 | 说明 |
|---|---|
| BLE 遥控 | `web_ctrl` 改协议无关层，命令集共用 |
| ESP32-S3 升级 | 代码零改动（只改 set-target）；相机走 DVP |
| 雷达 | 新组件经命令队列下发，motion 不动 |
| 视觉协处理器（RV1106） | UART 发高层指令，复用现有命令集 |
| IMU | 翻车自动检测（触发行为包，不入控制环） |
| 弹性肌腱弹跳 | 蝗虫 LA-MSA 机制拟真，低成本实验 |

## 已定案不做

3DOF 腿 / 增加舵机、丝滑全向行走、翻倒自复位、相机与深度学习（一期）、跳跃（一期）。

## 关键路径

```
M0 ──▶ M1 ──▶ M2 ──▶ M3 ──▶ M4
             ▲                  ▲
             └──── gait_sim ────┘   (扫参结果锁定 params.yaml)
             └── blender ──────────┘   (关键帧管线贯通 posture)
```

gait_sim 在 M2 之前必须可用：没有它就无法定 α/β/L1/L2，也无法确认步态可行性与滑移预算。