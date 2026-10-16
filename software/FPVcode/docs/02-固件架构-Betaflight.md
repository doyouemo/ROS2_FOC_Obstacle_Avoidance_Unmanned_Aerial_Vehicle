# Betaflight 固件架构导读

> 目的：给你一条**从 `main()` 走到电机转起来**的路径，让你能带着问题去读源码。
>
> **核对基准**：Betaflight master，版本 `2026.12.0-alpha`，
> commit **`41a5b43`**（2026-10-16）。本文档的行号已对着本地克隆
> `software/FPVcode/betaflight/` 逐项核对过。
> **若你拉到了更新的 master，行号会漂移**——用函数名搜索，别只认行号。

---

## ⚠️ 读之前必知：master 和你看到的教程不一样

网上大量教程（包括 YouTube 上的）讲的是 **Betaflight 4.4 / 4.5** 的结构。
master 上这几个**根本性变化**会让老教程直接误导你：

| 4.x 的做法 | master 的做法 |
|---|---|
| `make TARGET=DAKEFPVH743PRO` | **`make CONFIG=DAKEFPVH743PRO`** |
| 板级定义在 `src/main/target/<BOARD>/target.h` | **`src/main/target/` 下已无板级目录**，只剩 4 个公共头文件 |
| `make/targets.mk` 列 target | 改成 `mk/config.mk` + `mk/source.mk` |
| 板级定义在同一个仓库 | **拆到独立仓库 `betaflight/config`**，以 submodule 形式挂到 `src/config/` |
| `sensors/gyro.c` 一个文件 | 拆成 `gyro.c` + `gyro_init.c` + `gyro_filter_impl.c` |
| `dshot_bitbang.c` 在 `src/main/drivers/` | 挪到 **`src/platform/STM32/`** |
| CLI 在 `init.c` 里初始化 | CLI 改为**按需进入**（从 `msp_serial.c` 收到 `#` 时触发） |

**结论：先把 `make TARGET=...` 这类老习惯丢掉。**
本板的板级定义在 `betaflight/config` 仓库的 `configs/DAKE/DAKEFPVH743PRO/config.h`
（就是硬件文档里那份）。

---

## 1. 仓库布局

根目录：`Makefile`、`mk/`、`lib/`、`src/`、`images/`

| 路径 | 职责 |
|---|---|
| `src/main/` | 飞控应用本身，`main.c` 是入口 |
| `src/main/drivers/` | **硬件驱动**，不含飞行策略：`bus_spi.c`、`serial*`、`timer_*`、`dma.c`、`motor.c`、`dshot.c`、`max7456.c`、`flash/`、`accgyro/`、`rx/`（SPI 射频芯片） |
| `src/main/sensors/` | **传感器抽象 + 融合层**：`gyro.c`、`gyro_filter_impl.c`、`acceleration.c`、`barometer.c`、`battery.c`、`current.c`、`boardalignment.c` |
| `src/main/flight/` | **全部控制律**：`imu.c`、`pid.c`、`mixer.c`、`rpm_filter.c`、`failsafe.c`、`position.c` |
| `src/main/fc/` | 启动胶水 + **任务表**：`init.c`、`tasks.c`、`core.c`、`rc.c` |
| `src/main/scheduler/` | 协作式调度器：`scheduler.c` / `.h` |
| `src/main/rx/` | RC 链路协议 + 接收机抽象：`rx.c`、`crsf.c`、`sbus.c`、`expresslrs.c` |
| `src/main/io/` | 上层 IO 服务：`serial.c`、`gps.c`、`flashfs.c`、`ledstrip.c`、`piniobox.c`、`displayport_*.c` |
| `src/main/msp/` | 给 Configurator 用的 MSP 协议 |
| `src/main/blackbox/`、`osd/`、`telemetry/`、`cli/`、`cms/`、`pg/` | 日志 / OSD / 遥测 / CLI / 屏显菜单 / 参数框架 |
| `src/platform/` | **各 MCU 系列的后端**。STM32 相关：`STM32/dshot_bitbang.c`、`dma_stm32h7xx.c`、`link/`（链接脚本）、`mk/STM32H7.mk`、`target/STM32H743/` |
| `src/config/` | submodule，`make configs` 才拉下来（`betaflight/config` 仓库） |

**关键理解**：`src/main/` 是**与芯片无关的逻辑**，`src/platform/` 是**芯片相关实现**。
同一个 `motor.c` 在 H743 和 F411 上走的是 `src/platform/` 下不同的后端。

---

## 2. 启动流程

入口 **`src/main/main.c:65` 的 `main()`**，之后进死循环 `run()`（`:147`）→ `scheduler()`。

```
main()                                    main.c:65
├─ targetParseArgs()                      :68   （仅 USE_MAIN_ARGS 时）
├─ printfSerialInit()                     :75
├─ systemInit()                           :79   时钟/振荡器/PLL、Cache、MPU、基础 GPIO
├─ initPhase1()                           fc/init.c:296
│    ├─ tasksInitData()                          先把 task_attributes 绑到 tasks
│    ├─ initEEPROM() / readEEPROM()              参数存储
│    ├─ initBoardInformation() / pinioInit() / ledInit() / EXTIInit()
├─ initPhase2()                           fc/init.c:445
│    ├─ mixerInit() / motorDevInit()              混控与电机设备
│    ├─ targetBusInit() / configureSPIBusses()
│    ├─ sdioInitialize() / sdCardAndFSInit()
├─ [USB MSC 模式检查]                      main.c:92-100
│    └─ 若进 MSC 模式，initMsc() 后**永不返回**（不启动调度器）
├─ usbVcpInit()                           main.c:105  ← USB 在 phase1/2 之后、phase3 之前
├─ initPhase3()                           fc/init.c:656
│    ├─ adcInit() → initBoardAlignment() → gyroSetTargetLooptime()
│    ├─ gyroInit() (~:730)  gyroInitFilters() (~:740)
│    ├─ pidInit() (~:742)   mixerInitProfile() (~:744)
│    ├─ imuInit()  failsafeInit()  rxInit()
│    ├─ flashInit() + flashfsInit()
│    ├─ blackboxInit()   accStartCalibration() / gyroStartCalibration()
│    ├─ mspInit()  telemetryInit()
│    ├─ motorPostInit() + motorEnable()
│    └─ tasksInit() (~:1083)  ← 最后一步，任务表生效
└─ run() → while(true) scheduler();       main.c:147
```

**记住 `initPhase3` 里 `gyroInit()` 一定在 `pidInit()` 之前**——
这个顺序就是数据流顺序：**先有传感器，才有控制**。

---

## 3. 调度器与任务表 ★ 最重要的一张表

**如果要只读一个文件来理解整个飞控，就读 `src/main/fc/tasks.c`。**
`task_attributes[TASK_COUNT]`（**`tasks.c:380`**）就是整个固件的地图。

任务声明用 `DEFINE_TASK(...)` 宏；任务 ID、优先级枚举在 **`scheduler/scheduler.h`**
（优先级 `:70-78`，任务 ID `:107-220`，`task_attribute_t` `:222-230`，
周期宏 `TASK_PERIOD_HZ/MS/US` `:27-29`）。

| 任务 | 函数 | 周期 | 优先级 |
|---|---|---|---|
| `TASK_GYRO` | `taskGyroSample` | **125 µs = 8 kHz** | REALTIME |
| `TASK_FILTER` | `taskFiltering` | **125 µs** | REALTIME |
| `TASK_PID` | `taskMainPidLoop` | **125 µs** | REALTIME |
| `TASK_RX` | `taskUpdateRxMain` | 33 Hz（事件驱动） | HIGH |
| `TASK_DISPATCH` | `dispatchProcess` | 1000 Hz | HIGH |
| `TASK_MAIN` | `taskMain` | 1000 Hz | MEDIUM_HIGH |
| `TASK_ACCEL` | `taskUpdateAccelerometer` | 1000 Hz (H743) | MEDIUM |
| `TASK_ATTITUDE` | `imuUpdateAttitude` | **100 Hz** | MEDIUM |
| `TASK_SERIAL` / `TASK_OSD` / `TASK_TELEMETRY` | — | 100 Hz / 依 OSD 帧率 / 250–500 Hz | LOW |
| `TASK_BARO` / `TASK_COMPASS` / `TASK_GPS` / `TASK_CMS` | — | 各自 | LOW |

### 陀螺/PID 三件套是特例

它们**不走常规优先级搜索**。`scheduler.c` 里直接取 `TASK_GYRO` 跑，
然后用 **`gyroFilterReady()`**（`fc/core.c:1572`）和 **`pidLoopReady()`**（`fc/core.c:1581`）
判断这次采样要不要顺带跑滤波和 PID —— 判据是
`pidUpdateCounter % activePidLoopDenom`。

`pidUpdateCounter` 在 `taskGyroSample()`（`core.c:1562`）里每次陀螺采样自增一次。

### 环路频率怎么定的（H743）

- 陀螺采样率由 **`gyroSetSampleRate()`（`drivers/accgyro/gyro_sync.c:49`）** 决定。
  ICM42688P 落到 `default:` 分支 → **8000 Hz**，加速度计 1000 Hz
  （BMI270 = 3200 Hz，ICM20649 = 9000 Hz）
- **`gyroSetTargetLooptime()`（`sensors/gyro_init.c:812`）**：
  `targetLooptime = pid_process_denom × 1e6 / sampleRateHz`
- `DEFAULT_PID_PROCESS_DENOM = 1`（`flight/pid.c:92`），
  且 `STM32H743/target.h` **没有覆盖它**
  → **本板默认 8 kHz 陀螺 + 8 kHz PID**
  - `pid_process_denom = 2` → 8k/4k；`= 4` → 8k/2k（上限 16）

> 这是 **8kHz 控制环**。480MHz 的 M7 每周期有 60000 个时钟周期预算，
> 这就是为什么 H743 能跑 8k 而 F411 跑不动。你自己写的时候，
> 8kHz 意味着**陀螺读取 + 滤波 + PID + 混控 + DShot 全要在 125µs 内完成**。

---

## 4. 传感器流水线

### 驱动接口（`drivers/accgyro/accgyro.h`）

- **`gyroDev_t`**（`:107`）：`initFn` / `readFn` / `temperatureFn`、
  `gyroADCRaw[3]`、`gyroADC`、`scale`、`gyroZero[3]`、
  `gyroModeSPI`（`GYRO_EXTI_INT_DMA` 等）、`rotationMatrix`
- **`accDev_t`**（`:157`）：`initFn` / `readFn` / `acc_1G` / `ADCRaw[3]`
- 函数指针 typedef 在 `drivers/sensor.h:31-36`

**写一个 SPI 陀螺驱动要实现什么**：
一个 `bool xxxSpiGyroDetect(gyroDev_t*)`，它填入传感器型号、
设好 `initFn` / `readFn`，返回是否探测到；再实现这两个回调。

### 实际带起来的过程

1. `gyroInit()`（`sensors/gyro_init.c:640`）扫描 `GYRO_COUNT` 个传感器
2. `gyroDetect()`（`:360`）根据 `mpuDetectionResult.sensor` 分派
3. `gyroInitSensor()`（`:293`）：设对齐矩阵 → 调 `gyroSetSampleRate()`
   → **调 `gyroDev.initFn()`**（`:312`）→ 初始化滤波器
4. `gyroInitFilters()`（`:232`）：静态陷波 / LPF / 动态 LPF / slew limiter

**ICM42688P 的实例**：`drivers/accgyro/accgyro_spi_icm426xx.c`
（`icm426xxSpiGyroDetect` `:464`、`icm426xxSpiAccDetect` `:333`）
对照阅读：`accgyro_spi_bmi270.c`

### 滤波链（`sensors/gyro_filter_impl.c`，全文只有 ~95 行）

这个文件被 `gyro.c:483` 和 `:491` **`#include` 两次**（靠不同的 debug 宏区分），
所以是模板式写法。链路顺序：

```
降采样(均值 或 LPF2 抗混叠)
  → RPM 滤波        rpm_filter.c
  → 静态陷波 1 + 2   (SVF notch)
  → 低通 LPF1        (PT1 / PT2 / PT3 / SVF 可选)
  → 动态陷波         dyn_notch_filter.c
  → 写入 gyroADCf[]
```

之后 `gyroFiltering()`（`gyro.c:496`）再做溢出检查、yaw spin 检查，
以及给姿态环用的 PT1 平滑（`imuGyroFilter[]`）。

滤波原语在 `common/filter.c`：`pt1FilterApply`、`svfNotchFilter`、`biquadFilter` 等。

---

## 5. 姿态解算（`flight/imu.c`）

- 入口 **`imuUpdateAttitude()`（`:910`）**，由 `TASK_ATTITUDE` **100 Hz** 调用
- **`imuCalculateEstimatedAttitude()`（`:795`）**：算 `dt`、取滤波后的陀螺、
  算磁/GPS 航向误差、用 **`imuIsAccelerometerHealthy()`（`:352`，接受 0.9–1.1 g）** 判加速度计健康度
- **`imuMahonyAHRSupdate()`（`:243`）** —— 这就是那个互补滤波：
  1. 误差 `e` = 归一化实测加速度 × 估计重力方向（叉积，取自 `rMat` 的 Up 行）
  2. 积分项 `integralFB += imuDcmKi * e * dt`（超转速时跳过）
  3. `g += dcmKpGain * e + integralFB` —— **直接修正陀螺**
  4. 四元数积分 `q += 0.5*dt*ω⊗q` 后归一化
  - `dcmKpGain` 由 `imuCalcKpGain()`（`:472`）给出，随收敛**自动降增益**
- 欧拉角：`imuUpdateEulerAngles()`（`:329`），输出 `attitude.values.*`，单位 0.1°

> **这段就是你要复刻的东西**。它本质是个**带反馈的观测器**——
> 和你 `math_sim` 里的状态空间思维是同一套语言。

---

## 6. PID（`flight/pid.c`）

调用链：`subTaskPidController()`（`fc/core.c:1416`）→ **`pidController()`（`pid.c:1129`）**

每轴控制律（循环体 `:1322`，术语见 `common/axis.h`）：

| 项 | 实现 | 位置 |
|---|---|---|
| **设定值** | `getSetpointRate(axis)`，来自 `fc/rc.c`，2自由度（导数/前馈与反馈路径分离） | — |
| **误差** | `errorRate = setpoint - gyroADCf[axis]` | `:1397` |
| **P** | `Kp * errorRate * TPA` | `:1417` |
| **I** | `I = constrain(prevI + (Ki+accel) * dT * err, -limit, +limit)`，**靠 constrain 做抗积分饱和** | `:1422-1449` |
| **D** | 陀螺**先**过 `dtermNotch → dtermLowpass → dtermLowpass2`（`:1221-1223`），再 `delta = -(gyroDterm - prev) * pidFrequency`，支持 **D-max** | `:1491-1515` |
| **F 前馈** | `Kf * setpointDelta` | `:1533` |
| **S** | `getSterm()` | `:1061` |
| **合成** | `Sum = P + I + D + F + S`（未解锁或陀螺溢出时**全清零**） | `:1583, :1593-1610` |

**注意 D 项的关键设计**：D 作用于**陀螺的差分**而不是误差的差分，
且**先滤波再差分**。这是穿越机 PID 和教科书 PID 最大的差别，也是你从零写时最容易做错的地方。

---

## 7. 混控 → 电机（`flight/mixer.c`）

`subTaskMotorUpdate()`（`core.c:1502`）→ **`mixTable()`（`mixer.c:693`）**：

```
pidData[].Sum 限幅 ±pidSumLimit（yaw 单独）→ 除以 PID_MIXER_SCALING
  → 计算油门端点 / 油门限制 / thrust linearization / dyn-idle
  → motorMix[i] = roll*mix.roll + pitch*mix.pitch + yaw*mix.yaw
  → airmode / mixer 调整（LEGACY|LINEAR|DYNAMIC|EZLANDING）
  → applyMixToMotors()  写入 motor[] ∈ [0,1]
  → writeMotors() (mixer.c:98) → motorWriteAll() (drivers/motor.c:79)
```

混控表本身由 `mixer_init.c` 根据 `mixerConfig()->mixerMode` 建出来。

---

## 8. DShot 输出 —— H743 走位带（bitbang）

### 组帧（`drivers/dshot.c`）

**`prepareDshotPacket()`（`:123`）**：
```
packet = (value << 1) | requestTelemetry
csum   = XOR of 4 nibbles        （双向遥测时取反）
frame  = (packet << 4) | csum    → 16 bit
```

设备选择：**`motorDevInit()`（`drivers/motor.c:299`）** 决定走位带 DShot、
定时器 DMA DShot、还是模拟 PWM，并装上一张 vTable。

### 位带实现（`src/platform/STM32/dshot_bitbang.c`，862 行）

**机制**：一个"pacer"定时器（TIM1 或 TIM8）以 **3 倍符号率**自由运行；
它的 DMA 通道把一个 `uint32_t bbOutputBuffer[]` 循环送入 **GPIO BSRR**——
每个表项负责一个 state，把引脚拉高或拉低。
**同一个 GPIO 端口的多个电机共享一个表项**，所以**一条 DMA 流同时驱动该端口所有电机**。
`MOTOR_DSHOT_BUF_LENGTH = 16*3+3 = 51`。

关键函数：`dshotBitbangDevInit()` `:788`、`bbMotorConfig()` `:432`、
`bbWrite()` `:661`、`bbDMAIrqHandler()` `:313`、`bbSwitchToInput()`。

### ★ H743 的 D-cache / DMA 一致性 —— 你已经会遇到的坑

这正是 `03-从零写飞控路线图.md` §3 里提醒的那条，Betaflight 是这么处理的：

```c
#define BB_OUTPUT_BUFFER_ATTRIBUTE  DMA_RAM     // dshot_bitbang.c:73-75
// 且 aligned(32)
```

原因：**H7 的 D-cache 和 DMA 不保证一致**。所以：
- 输出缓冲区放 **DMA 可见的 RAM** 并 32 字节对齐
- 开 `USE_DSHOT_CACHE_MGMT` 时，每帧发前 `SCB_CleanDCache_by_Addr()`（`bbUpdateComplete` 附近）
- 输入（遥测）缓冲采集后要做 invalidate

**H7 用 DMAMUX，任意通道可服务任意请求**——这才让位带 DShot 能和陀螺/Flash 的 SPI DMA 共存。
冲突在**初始化时**就通过 `dshotBitbangGetStatus()` 暴露
（`DSHOT_BITBANG_STATUS_OK | _MOTOR_PIN_CONFLICT | _NO_PACER | _TOO_MANY_PORTS`，
枚举在 `drivers/dshot_bitbang.h:35-39`），而不是运行时数据错乱。

位带 DMA 中断优先级：`TASK_PRIORITY_REALTIME` + `NVIC_BUILD_PRIORITY(2,1)`。

### 双向 DShot 遥测 → RPM 滤波（闭环回喂）

```
bbDMAIrqHandler() (dshot_bitbang.c:313)
  → bbSwitchToInput()，同一条 DMA 改输入模式收电调回包
  → dshot_bitbang_decode.c 解码（GCR / 位定时）
  → updateDshotTelemetry() (dshot.c:264)
  → motorFrequencyHz[motor] 经 PT1 平滑 (dshot.c:305)
  → rpmFilterUpdate() (flight/rpm_filter.c:107)
  → rpmFilterRun() 在 gyro_filter_impl.c:52 被应用
```

> **注意调用时机**：`updateDshotTelemetry()` 在 **`taskFiltering()` 的最开头**被调
> （`core.c:1591`），就是为了让遥测在 `gyroFiltering()` 之前拿到最新值。
> 这种"数据新鲜度"的排布是实时系统设计的细节，值得学。

---

## 9. 接收机与失控保护

### 抽象层（`rx/rx.h`）

- **`rxRuntimeState_t`（`:183`）**：`rxProvider`、`serialrxProvider`、
  `channelCount`、以及**驱动 vtable**：`rcReadRawFn` / `rcFrameStatusFn` / `rcProcessFrameFn`
- 全局实例 `rxRuntimeState` 在 `rx/rx.c:141`
- **`rxInit()`（`rx.c:321`）** 根据 `FEATURE_RX_*` 分派；
  串口 → **`serialRxInit()`（`rx.c:241`）** 里一个大 switch
  （CRSF / SBUS / SRXL2 / Spektrum / IBUS / GHST / FPORT / MAVLINK…）
- 具体协议：`crsfRxInit()`（`rx/crsf.c:659`）、`crsfFrameStatus()`（`:510`）、`crsfReadRawRC()`（`:614`）

### 任务流

`TASK_RX` 用 `rxUpdateCheck()`（`rx.c:558`）做 checkFunc，
主体 `taskUpdateRxMain()`（`fc/tasks.c:194`）是个 **3 态状态机**：
`RX_STATE_CHECK` → `RX_STATE_MODES` → `RX_STATE_UPDATE`（调 `updateRcCommands()` + `updateArmingStatus()`），
并自测各态耗时让调度器能事件驱动而不是死等。

### 失控保护

- RX 侧：**`calculateRxChannelsAndUpdateFailsafe()`（`rx.c:834`）**，
  按通道配置 `AUTO`（回中+油门最低）/ `HOLD` / `SET` / `INVALID`
- 核心状态机：**`flight/failsafe.c`** —— `failsafeOnValidDataReceived()` /
  `failsafeOnValidDataFailed()` / `failsafeCheckDataFailurePeriod()`，
  处置策略 `AUTO_LANDING` vs **`DROP_IT`（直接掉油门）**

---

## 10. Blackbox 日志

- **`blackbox/blackbox.c`**：`blackboxUpdate()` 从 **`subTaskPidSubprocesses()`（`core.c:1470`）** 调用
  —— **在 PID 环里**，所以日志和 PID 同频
- 设备抽象 **`blackbox/blackbox_io.c`** 按 `device` 分派：
  - `BLACKBOX_DEVICE_FLASH` → **`io/flashfs.c`**（极简的**追加式**文件系统，
    只有顺序写 + 头标记，没有目录）
  - `BLACKBOX_DEVICE_SDCARD` → `io/asyncfatfs/asyncfatfs.c`
- 芯片驱动：`drivers/flash/flash.c` + **`flash_m25p16.c`**（本板 SPI3 / CS PA15）
- 擦除异步化在 `taskMain()` 里（`tasks.c:137` 调 `flashfsEraseAsync()`）——
  这是 8kHz 下 flash 日志能跑起来的关键

---

## 11. OSD

- **`drivers/max7456.c`**：`max7456Init()`（`:345`）、`max7456DrawScreen()`（`:629`，DMA 上传字符网格）
- **`io/displayport_max7456.c`**：`max7456DisplayPortInit()`（`:215`）填一张 `displayPortVTable_t`
- **`osd/osd.c`**：`osdInit()`（`:572`）、`osdUpdate()`（`:1435`）；
  具体元素在 **`osd/osd_elements.c`**（每个元素一个 `osdElementRender*`），
  警告在 `osd/osd_warnings.c`
- 选哪个 DisplayPort 在 `fc/init.c` 的 `initPhase3()`（~`:936-1010`），
  按 `osdConfig()->displayPortDevice` 决定（AUTO / MAX7456 / MSP / FrSkyOSD / FBOSD）

---

## 12. 编译与烧录（命令和教程不一样！）

需要 `arm-none-eabi-gcc` 在 PATH 上。

```sh
# 1) 一次性：拉板级配置 submodule（会去 clone betaflight/config）
make configs

# 2) 编本板固件 —— 三种写法等价
make CONFIG=DAKEFPVH743PRO
make DAKEFPVH743PRO
make DAKEFPVH743PRO_rev        # 文件名带上 git revision

# 其他
make CONFIG=DAKEFPVH743PRO clean
make CONFIG=DAKEFPVH743PRO flash     # 走 dfu_flash / tty_flash
make CONFIG=DAKEFPVH743PRO zip
make targets                          # 列出合法 MCU target
```

`TARGET=` 和 `CONFIG=` **互斥**（`mk/config.mk` 会直接报错）。

产物：`obj/betaflight_<version>_<BOARD>.hex` / `.bin` / `.uf2`，
ELF 在 `obj/main/betaflight_<BOARD>.elf`。

> **对你的直接意义**：这条路**必须装 GCC 工具链**，Keil 用不了。
> 想只读不编译的话，用 GitHub 上的官方 release hex 也行。

---

## 13. 推荐阅读顺序

从 `main()` 往里走，前 5 个读完就有全局观了：

1. **`src/main/main.c`** —— 80 行看完全部启动流程（配 `fc/init.c` 的三个 `initPhaseN`）
2. **`scheduler/scheduler.h`** —— 任务 ID、优先级、周期宏
3. **`fc/tasks.c`** —— `task_attributes[]` 这张表，**整个固件的地图** ★
4. **`scheduler/scheduler.c`** —— 实时任务怎么真正被派发
5. **`fc/core.c`** —— `taskGyroSample()` `:1562`、`taskFiltering()` `:1589`、
   `taskMainPidLoop()` `:1598`、`subTaskPidController()` `:1416`、`subTaskMotorUpdate()` `:1502`
   —— **心脏**
6. `drivers/accgyro/accgyro.h` → `sensors/gyro.c` → `sensors/gyro_init.c`
7. **`drivers/accgyro/accgyro_spi_icm426xx.c`** —— 一个完整的 SPI 陀螺驱动（你的型号）
8. **`sensors/gyro_filter_impl.c`** —— 整个滤波链就 95 行
9. **`flight/pid.c`** —— `pidController()` `:1129`，循环体 `:1322-1584`
10. **`flight/imu.c`** —— `imuMahonyAHRSupdate()` `:243`
11. **`flight/mixer.c`** —— `mixTable()` `:693`
12. `drivers/motor.c` + `drivers/dshot.c` → **`src/platform/STM32/dshot_bitbang.c`**
13. `drivers/dshot_bitbang_decode.c` + `flight/rpm_filter.c`
14. `rx/rx.h` + `rx/rx.c` + `rx/crsf.c` + `flight/failsafe.c`
15. `blackbox/blackbox_io.c` + `io/flashfs.c`；`osd/osd.c` + `drivers/max7456.c`

---

## 14. 对"自己写"最有价值的 5 个点

1. **先读 `fc/tasks.c` 那张表**——它告诉你一个飞控**必须有哪些任务、各自多快**。
   你自己排任务表时可以照这个抄。
2. **8kHz 环里的每一微秒都要算**——注意 Betaflight 把遥测更新塞在滤波前、
   把 flash 擦除丢到 1000Hz 的 `taskMain` 里，这些都是**实时性权衡**的教科书例子。
3. **D 项作用在陀螺差分上，且先滤波再差分**（`pid.c:1221`）——
   这是穿越机 PID 和教科书 PID 的核心差异。
4. **H7 的 D-cache/DMA 一致性**：看 `dshot_bitbang.c:73-75` 怎么用
   `DMA_RAM` 属性 + `SCB_CleanDCache_by_Addr()` 解决。
   你从零写时这一步做错，症状是"偶尔错一次、难复现"。
5. **双向 DShot 是闭环**：电调回传转速 → RPM 滤波 → 陀螺滤波。
   你的 FOC 电调要接进来，**这一环必须实现**，否则飞控端享受不到 RPM 滤波。
