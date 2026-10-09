# 机器蜘蛛 / Insect Spider

低成本四足机器蜘蛛，仿昆虫运动方式。硬件恒定（8 舵机），软件能力为主变量。

## 产品定位

- **形态**：4 条腿 × 2 自由度（髋：竖直轴前后扫摆；膝：水平轴抬放），8× MG996R
- **控制器**：ESP32-WROOM-32 + PCA9685（I2C 16 路 PWM）
- **遥控**：WiFi AP + 浏览器页面（虚拟摇杆 / 动作按钮 / 舵机标定），零 APP
- **供电**：2S 锂电 + UBEC 10A-Car（6.0V 档），独立舵机轨
- **机架**：3D 打印双层板，约 200×120mm，整机 ≤1.2kg
- **拟真原则**：多数昆虫无法翻身，机器人亦不自复位；缺省能力只做成本可接受的部分

### 明确不做（已定案）

| 项 | 原因 |
|---|---|
| 3DOF 腿 / 增加舵机 | 产品理念不堆成本；2DOF 够用 |
| 全向行走（丝滑横移/斜走） | 用「转向→直行→转回」软件宏替代 |
| 翻倒自复位 | 拟真一致性（叩头虫的弹跳翻身机制刻意不装） |
| 弹跳跳跃（一期） | 功率瓶颈；二期实验「弹性肌腱」机制拟真 |
| 相机 / 深度学习（一期） | 一期只做运动控制；相机留二期 |

## 硬件速览

| 项目 | 规格 |
|---|---|
| 主控 | ESP32-WROOM-32（`idf.py set-target esp32`） |
| 舵机驱动 | PCA9685，I2C GPIO21(SDA)/GPIO22(SCL)，400kHz，50Hz PWM，12bit |
| 舵机 | 8× MG996R，标称脉宽 1000-2000µs（批次差异见 docs/hardware.md） |
| 腿段 | 大腿 L1=100mm，小腿 L2=110mm（参数化，见 tools/gait_sim/params.yaml） |
| 机身 | 双层板 200×120mm，层间距 30mm，外八前 36°/后 48°（见 docs/gait_design.md §3.5） |
| 站立高度 | 可调 90~150mm，巡航目标 120mm（膝部留约 30mm 抬脚行程） |
| 重量预算 | 舵机 440g + 框架 250~350g + 电池 130~160g + 电子 90g + 线材 80g ≈ 1.0~1.1kg |
| 舵机供电 | UBEC 10A-Car（输出 **6.0V** 档）+ 1000µF 母线电容 |
| 主控供电 | DC-DC 5.0V → ESP32 **VIN**，与舵机轨**完全独立**（避免 brownout） |
| PCA9685 | VCC=**3.3V**（接 ESP32 3.3V）/ V+=**6.0V**（接 UBEC），**禁止互连** |

扭矩校核（最恶劣工况：Trot 双腿支撑 + 抬机身）：
`单腿 0.6kg × 力臂 55mm ≈ 3.3 kg·cm` vs `MG996R@6V 11 kg·cm`，余量约 3 倍。

## 仓库结构

```
spider_robot/
├── README.md               本文
├── build.sh                构建封装（自动加载 IDF 环境）
├── partitions.csv          4MB 分区表（OTA 双槽 + nvs 20KB）
├── sdkconfig.defaults      默认配置
├── docs/                   设计文档（先看 hardware.md 与 architecture.md）
├── main/                   应用入口与装配
│   ├── main.c              启动横幅、几何派生量打印、NVS 初始化
│   └── Kconfig.projbuild   I2C 引脚、PWM 频率、tick、急停脚、PCA9685 OE 脚等
├── components/
│   ├── kinematics/         2DOF 逆解（已实现，含主机端测试）
│   │   ├── include/spider_params.h       ← gen_params.py 生成
│   │   └── test/test_kinematics.c        ← 15 项断言
│   ├── pca9685/            PCA9685 I2C 驱动（寄存器级）        [M1 已实现]
│   │   ├── include/spider_pca9685_math.h   纯算术（分频/脉宽换算）
│   │   └── test/test_pca9685.c            ← 18 项断言
│   ├── servo/              舵机层：标定、限位、斜坡、NVS      [M1 已实现]
│   │   ├── include/spider_servo_logic.h     纯逻辑（无 I2C/NVS 依赖）
│   │   └── test/test_servo.c                ← 11 项断言
│   ├── posture/            姿态与关键帧播放器 + 上传解析       [M2/M4]
    │   │   ├── include/spider_kf_parse.h   上传校验（纯逻辑）
    │   │   ├── src/kf_build.c            kf_check_meta / kf_set_frame
    │   │   ├── src/kf_parse.c            cJSON 适配层（需 IDF）
    │   │   └── test/test_kf_build.c       ← 61 项断言
    │   ├── gait/               步态引擎（爬行 4 拍 / Trot 2 拍）    [M2]
    │   ├── motion/             运动任务：命令队列、斜坡、侧移宏      [M2]
    │   └── web_ctrl/           WiFi AP + httpd + WebSocket + 内嵌控制页 [M3 已实现]
    │       ├── src/web.c         AP / HTTP / WS / 10Hz 遥测
    │       ├── src/ws_cmd.c      JSON 命令 -> motion（docs/protocol.md §3）
    │       └── index.html        控制页（22.8KB → gzip 8.7KB 内嵌）
    └── tools/
        ├── gen_params.py       params.yaml -> spider_params.h + 自洽校验
        ├── check_partition.py  app 分区余量检查（build.sh 自动调用）
        ├── protocol_check.py   文档 ↔ 实现一致性检查
        ├── run_tests.sh        主机端单元测试（147 项断言）
        ├── gait_sim/           步态预览器
        │   └── params.yaml     参数唯一定义源
        ├── blender/            【M4 待建】参数化骨架 + bpy 导出 + JSON→C
        └── kf_convert.py       【M4 待建】Blender JSON 序列 → C 头文件
```

## 里程碑

- **M0** ✅ 环境与骨架（IDF v5.5.1、`esp32` 目标、编译通过、参数生成器与主机端测试可用）
- **M0.5** ✅ 硬件规格文档（见 docs/hardware.md）
- **M1** ✅ 舵机硬件层（pca9685 + servo）代码完成，主机端断言通过，**待实机 `cal scan` 验证通道对应与极性**
- **M2** ✅ 运动层（kinematics / gait / posture / motion / cli）代码完成，**待实机行走验证**
- **M3** ✅ Web 遥控（WiFi AP + WebSocket + 控制页）代码完成，**待实机联调**
- **M3.5** ✅ 关键帧上传协议（`keyframe_upload`，含 61 项解析器断言）
- **M4** 联调收尾（装配、标定、调参、README 完善）+ Blender 导出管线

> 全部主机端测试 `./tools/run_tests.sh`：**147 项断言**（含 protocol_check 10 项、
> params_check 21 项、gait_sim 各 17 项）。

## 构建

需要 ESP-IDF **v5.5.1**（已安装于 `~/env/esp/esp-idf-v5.5.1`）。`build.sh` 自动探测并加载：

```bash
./build.sh                 # 等价 idf.py build（并检查 app 分区余量）
./build.sh flash monitor   # 烧录 + 串口监视
./build.sh menuconfig      # Kconfig 配置
./build.sh bench on|off    # 单腿台架模式（通电不驱动舵机）
./build.sh size            # 内存占用
./tools/run_tests.sh       # 全部主机端测试（不烧板）
```

> `bench on` 会清空 `build/` 与 `sdkconfig`——IDF 只在首次生成时应用
> `sdkconfig.defaults`，不清就会烧进旧固件。用于装配前测机械起止角，
> 见 [docs/calibration.md](docs/calibration.md) §0.5。

IDF 定位顺序（`build.sh` 的 `find_idf`）：

1. `IDF_PATH_DIR` / `IDF_PATH` 环境变量
2. `~/env/esp/esp-idf-v5.5.1`
3. `~/esp/esp-idf`（已删除，保留为回退候选）
4. PATH 上 `idf.py` 所在树

换 IDF 版本或换机器时无需改脚本。脚本每次会打印实际选中的路径与版本。

> 从 v5.4.1 迁到 v5.5.1 零代码改动：同一条小版本线，工具链同为
> `xtensa-esp-elf esp-14.2.0_20241119`，26 个 Kconfig 符号与 6 个关键 API 全部在位。

### 参数生成流程

```
tools/gait_sim/params.yaml   ← 几何/步态/时序参数唯一数据源（手改这里）
        │  python3 tools/gen_params.py
        ▼
components/kinematics/include/spider_params.h   ← 自动生成，勿手改
```

生成器会先跑 19 项自洽校验（巡航高度、软限位、步幅/摆角、速度区间等），**任一失败则不写文件**。`meta.freeze_status` 为 `pending_sweep` 时只给 WARN——表示 gait_sim 扫参尚未完成，不可据此出打印件。

### 手机连控（实机）

烧录后 ESP32 开热点 `SpiderRobot-XXXX`，手机连上后浏览器访问 `http://192.168.4.1`。
控制页单文件内嵌（gzip），无外部依赖、离线可用。

**首次使用顺序**（顺序错了会顶死舵机）：
1. **架空机器人**（脚不着地）
2. 串口：`cal enter` → `cal scan` → 确认 FL/FR/RL/RR 对应与极性 → `cal save`
3. 落地：`pose stand`，静置观察 30s 无异常
4. 手机打开控制页，先急停按钮试一遍，再低档位行走

### 主机端测试

`kinematics` 是纯计算组件（不依赖 I2C / FreeRTOS），可直接在 macOS 上编译运行，验证几何自洽与软限位：

```
$ ./tools/run_tests.sh
=== test_kinematics ===   15 项  几何自洽、软限位、步幅↔摆角、变高耦合比
=== test_servo ===         11 项  标定合法性、端点锚定映射、reverse/trim、斜坡
=== test_pca9685 ===       18 项  50Hz prescale=122、脉宽↔计数、越界拒绝
=== test_gait ===          38 项  零滑移、抬脚、速度、转向、变高、防翻约束
=== test_kf_build ===      61 项  关键帧上传校验（名字/帧数/8 关节/时间单调/ease）
=== protocol_check ===     10 项  文档 ↔ WS/CLI 实现的协议一致性
=== params_check ===       21 项  params.yaml 几何自洽 + 支撑多边形 + 三足裕度
=== gait_sim crawl/trot === 17×2 项  仿真验收（与固件同源参数）
... 共 147 项断言，全部 PASS
```

> `protocol_check` 是为拦住「文档写了、实现漏了」这类漂移而加的。
> 这个项目已经因此被咬过两次（腿位方位角配错、抬脚超限），
> 第三次改成构建期自动比对。

步态预览（打印 3D 件前必过）：
```bash
python3 tools/gait_sim/preview.py --check-only          # 验收
python3 tools/gait_sim/preview.py                        # 3D 动画
python3 tools/gait_sim/preview.py --sweep alpha beta_cruise   # 扫参
```

设计上把**纯逻辑与硬件 I/O 严格分离**（`servo_logic.c` / `pca9685_math.c` / `kinematics.c` / `gait.c` / `kf_build.c` 均不依赖 I2C/NVS/FreeRTOS），因此这些断言无需烧板即可跑完。

> 新增组件后请 `rm -rf build` 再编译：增量构建会静默跳过尚未被 CMake 识别的组件文件。

## 文档索引

| 文档 | 内容 |
|---|---|
| [docs/hardware.md](docs/hardware.md) | 尺寸包络、重量预算、关节轴向、供电接线、标定说明、3D 打印件清单 |
| [docs/architecture.md](docs/architecture.md) | 组件职责、FreeRTOS 任务、命令流、坐标系与角度约定、NVS 布局 |
| [docs/protocol.md](docs/protocol.md) | WebSocket / 串口指令集、JSON 字段、参数定义 |
| [docs/gait_design.md](docs/gait_design.md) | 步态几何、时序、滑移预算、变高策略、防翻约束 |
| [docs/calibration.md](docs/calibration.md) | 装配顺序、逐机标定流程、验收 |
| [docs/insect_behavior.md](docs/insect_behavior.md) | 拟真行为包（假死、挣扎、蓄力蹲） |
| [docs/roadmap.md](docs/roadmap.md) | 里程碑 M0~M4 与验收标准 |
| [tools/gait_sim/params.yaml](tools/gait_sim/params.yaml) | 几何/步态/时序参数唯一定义源 |
