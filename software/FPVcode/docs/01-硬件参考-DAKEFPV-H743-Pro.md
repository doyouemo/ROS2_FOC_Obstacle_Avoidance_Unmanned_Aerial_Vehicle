# DAKEFPV H743 Pro 硬件参考

> 目标：在**没有官方原理图**的前提下，得到一份可信、可验证的硬件真值表。
> 方法：用三份独立的开源固件板级定义交叉验证，再用 SWD 实测收口。
>
> 状态：**引脚映射可信度高（三方一致）**；**传感器型号与标定参数必须在实物上确认**（见 §6）。

---

## 0. 板子身份

| 项 | 值 |
|---|---|
| 厂商 | DAKEFPV（大科），厂商 ID `DAKE` |
| 型号 | H743 Pro |
| MCU | STM32H743VIT6，Cortex-M7 @480MHz |
| 外部晶振 | 8 MHz（`OSCILLATOR_HZ 8000000`） |
| 形态 | 30.5×30.5 mm 飞塔 |
| Betaflight target | `DAKEFPVH743PRO` |
| ArduPilot 板级 | `DAKEFPVH743Pro` |
| PX4 board_id | 1193（**未合入上游**，见 §5.7） |

同一厂商还有 `DAKEFPVH743`、`DAKEFPVH743_SLIM`、`DAKEFPVF405`、`DAKEFPVF722`、`DAKEFPVF411`、`DAKEFPVF435` 等 target，**别搞混**——引脚完全不同。

### 三份权威来源

| 来源 | 路径 | 用途 |
|---|---|---|
| Betaflight target config | `betaflight/config` → `configs/DAKE/DAKEFPVH743PRO/config.h` | 引脚、timer/DMA、默认参数 |
| ArduPilot hwdef | `ardupilot/libraries/AP_HAL_ChibiOS/hwdef/DAKEFPVH743Pro/hwdef.dat` | **最完整的定时器/引脚/模拟量定义** |
| PX4 板级 | PR #27389 → `boards/dakefpv/h743/` | 第三方交叉验证（未合入） |

> ⚠️ **路径变更**：Betaflight config 仓库在 2026-07-24 后改成按厂商分组，
> 旧路径 `configs/<board>/` 已 404，新路径是 `configs/DAKE/<board>/`。

> ⚠️ **本地资料不可直接套用**：`reference/H743飞控/` 里的 Urmouky H743 经查是
> **Matek H743-WING 的引脚兼容复刻**（hwdef 头行注明 `for Matek H743-WING`，
> `APJ_BOARD_ID 1013`）。它和本板同为 H743 但**引脚完全不同**，
> 只能当"H743 飞控长什么样"的参考，**不能当本板真值**。

### 0.1 大科三个 H743 版本怎么区分 ★

大科有 **3 个 H743 型号**，引脚差异很大。**先确认你手上是哪块**，否则后面全错。

| | `DAKEFPVH743`（基础版） | `DAKEFPVH743PRO`（本文档对象） | `DAKEFPVH743_SLIM` |
|---|---|---|---|
| **M5–M8** | PD12 PD13 PD14 PD15 | **PE9 PE11 PC8 PC9** | PA0 PA1 PA2 PA3 |
| **S1–S4** | PE5 PE6 PC8 PC9 | **PD12 PD13 PD14 PD15** | PD12 PD13 PD14 PD15 |
| LED 灯带 | PE9 | **PB0** | PB0 |
| 摄像头切换 | PB8 | **PE5** | PC8 |
| 蜂鸣器 | PC2 | **PC2** | PE10 |
| UART4 | PD1 / PD0 | **PB8 / PB9** | PB9 / PB8 |
| **SD 卡槽** | 无 | **无** | **有（SDIO）** |
| 电压采样 | PC1 | **PC1** | **PA4**（不一样！） |
| GYRO_1 CLKIN | PB9 | **PE6** | PB7 |

**三个版本相同的引脚**（所以它们**不能**用来区分型号）：
`FLASH_CS=PA15`、`MAX7456_CS=PB12`、`GYRO_1_CS=PA4`、`GYRO_1_EXTI=PC4`、
`GYRO_2_CS=PB1`、`GYRO_2_EXTI=PB2`、`I2C2=PB10/PB11`、`ADC_CURR=PC0`、`ADC_RSSI=PC5`

**最快的区分方法（按顺序判断）：**

1. **板上有没有 SD 卡槽？**
   有 → 是 `DAKEFPVH743_SLIM`；没有 → 继续
2. **看电机焊盘丝印：`PD12–PD15` 标的是 `M5–M8` 还是 `S1–S4`？**
   标 `S1–S4` → `DAKEFPVH743PRO`
   标 `M5–M8` → 基础版 `DAKEFPVH743`
3. 或者直接看板子正面丝印上的型号名

> 本文档全文针对 **`DAKEFPVH743PRO`**。若实测发现是另外两个版本，
> 引脚表需要按上表重写。

---

## 1. 总线与片选总表

三方（BF / AP / PX4）**完全一致**，可信度最高。

| 总线 | 时钟 | MISO | MOSI | 挂载设备 | CS | 备注 |
|---|---|---|---|---|---|---|
| **SPI1** | PA5 | PA6 | PA7 | IMU #1 | **PA4** | DRDY/EXTI = **PC4** |
| **SPI2** | PB13 | PB14 | PB15 | OSD (AT7456E) | **PB12** | MODE0，10 MHz |
| **SPI3** | PC10 | PC11 | PC12 | Blackbox Flash | **PA15** | MODE3，见 §4 注意 PA15=JTDI |
| **SPI4** | PE12 | PE13 | PE14 | IMU #2 | **PB1** | DRDY/EXTI = **PB2** |
| **I2C2** | PB10 (SCL) | — | PB11 (SDA) | 气压计（+ 外接磁罗盘） | — | 板上**无内置磁罗盘** |

**UART（8 路，三方引脚一致）**

| 口 | TX | RX | 建议用途（AP 默认） |
|---|---|---|---|
| USART1 | PA9 | PA10 | GPS1 |
| USART2 | PD5 | PD6 | 遥测 / MSP（BF `MSP_UART`） |
| USART3 | PD8 | PD9 | 电调遥测（BF `ESC_SENSOR_UART`） |
| UART4 | PB8 | PB9 | MSP DisplayPort（数字图传 OSD） |
| **UART5** | PB6 | **PB5** | **接收机 RCIN** ← 接 ELRS/CRSF 用这路 |
| USART6 | PC6 | PC7 | 用户 |
| UART7 | PE8 | PE7 | 用户 |
| UART8 | PE1 | PE0 | 用户 |

> UART5 的 RX 支持所有单向串口 RC 协议；SBUS 已硬件反相。FPort/SRXL2 需要外部双向反相器（`SERIAL5_OPTIONS=4`）。

**CAN**：只有 ArduPilot 定义了 —— `PD0` (CAN1_RX) / `PD1` (CAN1_TX)，另有 `PD2` 作 silent 控制。
Betaflight 和 PX4 都**没有** CAN 定义。AP wiki 宣称本板有 "1x CAN port"。
→ 结论：**CAN 引脚存在但只有 AP 实现**，焊盘是否引出**需实物确认**（见 §6）。

---

## 2. 引脚映射（电机 / 舵机 / 外设）

以 ArduPilot hwdef 为准（它明确标了定时器通道），Betaflight 的 `TIMER_PIN_MAPPING` 交叉印证。

| 输出 | 引脚 | 定时器通道 | DMA | 能力 |
|---|---|---|---|---|
| M1 | PA0 | TIM2_CH1 | ✓ | **双向 DShot** |
| M2 | PA1 | TIM2_CH2 | ✓ | — |
| M3 | PA2 | TIM2_CH3 | ✓ | **双向 DShot** |
| M4 | PA3 | TIM2_CH4 | ✓ | — |
| M5 | PE9 | TIM1_CH1 | ✓ | **双向 DShot** |
| M6 | PE11 | TIM1_CH2 | ✓ | — |
| M7 | PC8 | TIM8_CH3 | ✓ | **双向 DShot** |
| M8 | PC9 | TIM8_CH4 | ✓ | — |
| S1 | PD12 | TIM4_CH1 | ✓ | 仅 PWM |
| S2 | PD13 | TIM4_CH2 | ✓ | 仅 PWM |
| S3 | PD14 | TIM4_CH3 | ✗ NODMA | 仅 PWM |
| S4 | PD15 | TIM4_CH4 | ✗ NODMA | 仅 PWM |
| PWM13 | PE5 | TIM15_CH1 | ✗ NODMA | **摄像头切换** |
| PWM14 | PB0 | TIM3_CH3 | ✓ | **LED 灯带（WS2812）** |

分组（同一组的通道**必须同频同协议**，这对 DShot 很重要）：
`M1-4` / `M5,M6` / `M7,M8` / `S1,S2` / `S3,S4` / `PE5` / `PB0`

**其他引脚**

| 功能 | 引脚 | 说明 |
|---|---|---|
| 蜂鸣器 | PC2 | **反相驱动**（BF `BEEPER_INVERTED`） |
| ADC 电压 | PC1 | ADC1_CH11 |
| ADC 电流 | PC0 | ADC1_CH10 |
| ADC RSSI | PC5 | ADC1_CH8 |
| LED0 (蓝) | PD10 | |
| LED1 (绿) | PD11 | |
| LED2 (红) | PA8 | |
| PINIO1 | PE2 | 摄像头切换（HIGH=CAM1，LOW=CAM2） |
| PINIO2 | PE3 | **12V BEC 使能**（12V 图传供电） |
| PINIO3 | PE4 | 蓝牙 RF 开关 |
| SWD | PA13 / PA14 | 调试口 |
| USB | PA11 (DM) / PA12 (DP) | |

---

## 3. 电源与模拟

| 项 | 值 | 来源 |
|---|---|---|
| 输入电压 | 4S–12S LiPo | AP wiki |
| BEC 3.3V | 0.5 A | AP wiki |
| BEC 5V | 3 A | AP wiki |
| BEC 12V | 3 A（GPIO 控制，PE3） | AP wiki |
| 电压采样 | 最高 12S | AP wiki |
| 电流采样量程 | 最高 130 A | AP wiki |
| 电压 scale | **16.0** | BF 160÷10 / AP 16.0 —— **一致** ✓ |
| 电流 scale | BF `120` vs AP/PX4 `83.3` | **不一致** ✗ 见 §6.4 |

---

## 4. Timer / DMA 要点

Betaflight 的 `TIMER_PIN_MAPPING` 四列含义是 **索引 / 引脚 / occurrence / dmaopt**
（`#define TIMER_PIN_MAP(i, p, o, d)`，见 `src/main/pg/timerio.c`），
它**不直接写定时器号**——定时器由芯片的 AF 复用表推导。

- 8 路电机分到 **dmaopt 0–7**，一路一个 DMA 流
- `PB0`（LED 灯带）用 **occurrence 2、dmaopt 8**
- `DEFAULT_DSHOT_BITBANG = DSHOT_BITBANG_ON` —— 默认走**位带（bitbang）+ DMA** 而不是定时器硬件 DShot
- `ADC1_DMA_OPT 9` / `ADC3_DMA_OPT 10`

### ⚠️ PA15 是 JTDI
`PA15` 是 JTAG 的 JTDI 引脚，被复用成 Flash 片选。SWD 调试时 PA13/PA14 是独立的，
不受影响；但**如果你的调试器走 JTAG（5 线）而不是 SWD（2 线），会冲突**。用 SWD。

---

## 5. 三方交叉验证结果

### 5.1 完全一致 ✓
MCU、SPI 总线与片选、8 路 UART 引脚、OSD、Blackbox Flash、ADC 三路引脚、
蜂鸣器、三个 LED、三个 PINIO、I2C2、无内置磁罗盘、电压 scale 16.0

### 5.2 IMU 型号 —— **必须实测**
| 来源 | 声称支持 |
|---|---|
| 厂商标称 | 双 ICM42688P |
| Betaflight | MPU6000、ICM42688P、LSM6DSK320X、BMI270（4 种都编进去） |
| ArduPilot | Invensensev3（=ICM42688）、LSM6DSV、BMI270 自动探测 |
| PX4 | 只启用 ICM42688P |

板子有**多个版本**（BF 有个提交 2026-08-13 把 LSM6DSV16X 换成了 BMI270）。
**config 里开的是"超集"，不代表你板上焊的是哪颗。**
→ 最可能是 **ICM42688P**，但要用 WHO_AM_I 寄存器确认（见 §6.1）。

### 5.3 气压计 —— 结论：**SPL06**
- ArduPilot：`BARO SPL06 I2C:0:0x76`（只启用 SPL06）
- PX4：`CONFIG_DRIVERS_BAROMETER_GOERTEK_SPL06=y`，`spl06 start -X -b 2`
- Betaflight config：只开了 `BMP280` 和 `DPS310`，**没有 SPL06 这一行**

两家独立来源都指向 SPL06，且 I2C 地址 0x76 一致 → **BF 那边大概率是漏写了 SPL06 驱动**。
结论：**SPL06，I2C2，地址 0x76**，挂在 `PB10/PB11`。

> 补强证据：`DAKEFPVH743`、`DAKEFPVH743PRO`、`DAKEFPVH743_SLIM` **三个 BF config
> 全都只写 BMP280 + DPS310、全都没有 SPL06**。这说明是 DAKEFPV 系列 config 的
> **系统性遗漏**，而不是某块板的特例 —— 更支持"实际是 SPL06"的结论。
> 但仍以实测为准。

### 5.4 电流标定 —— 冲突
BF `120` vs AP/PX4 `83.3`。**两边单位约定本就不同，数字不能直接比**，
且分流电阻有公差。→ **一律以实测标定为准**（见 §6.4），不要信任何一边的默认值。

### 5.5 输出命名冲突（引脚一致，叫法不同）
PX4 把 `PD12–PD15` 叫 M5–M8、把 `PE9` 叫 "LED"；
ArduPilot/Betaflight 把 `PD12–PD15` 叫 S1–S4、把 `PE9` 叫 M5。
**焊盘上的物理引脚是同一批**，只是编号体系不同。
→ 接线时**认引脚丝印，不要认"M几"**。

### 5.6 IMU 朝向 —— 三个来源两两不一致
| 来源 | IMU1 (SPI1) | IMU2 (SPI4) |
|---|---|---|
| Betaflight | `CW0_DEG` | `CW180_DEG` |
| ArduPilot | `ROTATION_ROLL_180` | `ROTATION_PITCH_180` |
| PX4 | `-R 8` (ROLL_180) | `-R 12` (PITCH_180) |

AP 和 PX4 一致（ROLL_180 / PITCH_180），BF 的两个值按它自己的偏航约定写法不同。
→ **以 AP/PX4 为准的可能性大，但仍需实测验证**（见 §6.3）。
双 IMU 融合前，两路姿态**必须统一到同一个机体坐标系**，这里是第一个坑。

### 5.7 Flash 分区（写自己固件时很关键）

**出厂（Betaflight）的实际布局** —— 直接来自
`src/platform/STM32/link/stm32_flash_h743_2m.ld` 的注释块，不是推测：

```
0x08000000 – 0x0801FFFF   128K   bootloader / 向量表 + startup
0x08020000 – 0x0803FFFF   128K   config（参数存储，__config_start~__config_end）
0x08040000 – 0x081FFFFF  1792K   firmware
```

H743 是 2 MB Flash、**128 KB 一个扇区**，共 16 个扇区。Betaflight 把它切成
「128K bootloader + 128K 参数区 + 1792K 固件」，参数不再挤在最后一个扇区
（这是新的 `CONFIG_IN_FLASH` 方案）。

**三方互相冲突：**

| 方案 | 保留区 | 落到哪个地址 |
|---|---|---|
| **Betaflight（出厂）** | 128K bootloader | 固件从 `0x08040000` 起 |
| PX4 | `APP_LOAD_ADDRESS 0x08020000` | **正好压在 BF 的参数区上** |
| ArduPilot | `FLASH_RESERVE_START_KB 384` | 固件从 `0x08060000` 起 |

> ⚠️ **bootloader 和固件必须成对使用。** 三方各自的分区互不兼容——
> 刷 PX4 要连它自己的 bootloader 一起刷，不能只换应用。
>
> ⚠️ **你写自己的固件时**，如果保留出厂 bootloader，**链接地址必须是 `0x08040000`**，
> 且不要碰 `0x08020000` 那 128K（除非你确定不要 Betaflight 的参数了）。
> 烧错地址会把 bootloader 冲掉，板子变砖（只能靠 DFU 救）。

### 5.8 出厂默认串口分工
| | 接收机 | 遥测/MSP | 电调遥测 |
|---|---|---|---|
| Betaflight | UART5 | USART2 (MSP) | USART3 |
| ArduPilot | UART5 (RCIN) | UART4 (DisplayPort) | USART3 |

---

## 6. SWD 实测清单（拿到板子第一件事）

用 STLink/JLink 挂 SWD（**PA13/PA14，SWD 模式，不要用 JTAG**）。
以下每一项都是上面"存疑"项的收口动作，做完这份硬件表才算真值。

### 6.1 确认 IMU 型号
1. 先用 ST-Link Utility / STM32CubeProgrammer **读回整片 Flash** 存到 `dump/`，作为出厂备份（**先备份，再动手**）
2. 上电，用调试器读 IMU 的 `WHO_AM_I`：
   - ICM42688P：寄存器 `0x75` → 期望 `0x47`
   - BMI270：寄存器 `0x00` → 期望 `0x24`
   - MPU6000：寄存器 `0x75` → 期望 `0x68`
   - LSM6DSV16X：寄存器 `0x0F` → 期望 `0x70`
3. 两颗都要读，确认是**同型号**还是**混装**

### 6.2 确认气压计
读 I2C2 上 `0x76`。SPL06 的 `PRS_CFG`/芯片 ID 寄存器可读；
也可以直接**扫描 I2C2 总线**列全部地址，看实际挂了什么（可能还有外接磁罗盘预留）。

### 6.3 确认 IMU 朝向
用调试器读静止状态下的**加速度计三轴**：水平放置时，
正确朝向应当只有 **Z 轴 ≈ ±1g**，X/Y ≈ 0。
逐个倾斜 90° 验证符号。这一步做完，双 IMU 融合才有基础。

### 6.4 标定电压/电流
- 电压：万用表量电池电压，读 ADC 原始值，反算分压比（不要信 16.0，要实测）
- 电流：接已知负载（或电子负载），读 ADC 原始值 → 算出 A/V
- **注意**：板上电流采样可能是双向的，零电流时的 offset 也要标

### 6.5 确认 CAN 焊盘
万用表通断档，量 `PD0`/`PD1` 是否引到外部焊盘。若没引出，CAN 这条路直接放弃。

### 6.6 确认 Flash 分区
读回 Flash 后看：
- bootloader 占几个扇区（通常开头一段非 `0xFF`）
- Betaflight 固件从哪个地址开始
这决定你**自己写的固件能从哪里开始烧**。

---

## 7. 一页速查

```
MCU      STM32H743VIT6 @480MHz, 8MHz 晶振, 2MB Flash
IMU      SPI1 CS=PA4 DRDY=PC4  |  SPI4 CS=PB1 DRDY=PB2   (型号待实测)
OSD      SPI2 CS=PB12 (AT7456E)
Flash    SPI3 CS=PA15 (16MB, 注意=JTDI)
气压计    I2C2 @0x76 (SPL06)  PB10=SCL PB11=SDA
电机      PA0 PA1 PA2 PA3 PE9 PE11 PC8 PC9   (M1..M8)
舵机      PD12 PD13 PD14 PD15                 (S1..S4)
LED灯带   PB0          摄像头切换 PE5
接收机    UART5  (PB5=RX, PB6=TX)
电调遥测  USART3 (PD8/PD9)
电压采样  PC1 (scale 待标定, 参考 16.0)
电流采样  PC0 (scale 待标定, BF=120 / AP=83.3 冲突)
蜂鸣器    PC2 (反相)   LED: PD10 蓝 / PD11 绿 / PA8 红
12V BEC   PE3         摄像头切换 PE2
SWD       PA13/PA14
```
