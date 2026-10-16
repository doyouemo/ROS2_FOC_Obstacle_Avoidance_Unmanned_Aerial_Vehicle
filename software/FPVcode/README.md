# FPVcode — 穿越机飞控学习工作区

学习对象：**DAKEFPV（大科）H743 Pro** 飞控。三条线：硬件 → 固件架构 → 自己写。

## 目录

| 文档 | 内容 |
|---|---|
| [docs/01-硬件参考-DAKEFPV-H743-Pro.md](docs/01-硬件参考-DAKEFPV-H743-Pro.md) | 引脚总表、总线、电源、定时器/DMA、**SWD 实测清单** |
| [docs/02-固件架构-Betaflight.md](docs/02-固件架构-Betaflight.md) | Betaflight master 架构导读 + 15 个文件的阅读顺序 |
| [docs/03-从零写飞控路线图.md](docs/03-从零写飞控路线图.md) | Stage 0→8 分阶段路线、H743 专属坑、与 FOC/ROS2 目标的衔接 |

| 目录 | 用途 |
|---|---|
| `code/` | 自己写的飞控工程（待建） |
| `docs/` | 上述分析文档 |
| `notes/` | 随手笔记、实测记录 |
| `dump/` | 出厂 Flash 备份、固件 dump |
| `betaflight/` | **Betaflight 主仓**（浅克隆，`2026.12.0-alpha` @ `41a5b43`） |
| `betaflight-config/` | **板级配置仓**，本板的 `configs/DAKE/DAKEFPVH743PRO/config.h` 在此 |

> 两个上游仓已加进 `.gitignore`，不会入库。
> 浅克隆没有历史（`git log` 只有 1 条）；要看提交历史执行
> `git -C betaflight fetch --unshallow`（约 1GB，慢）。

## 当前状态（2026-10-16）

**已完成**：硬件真值表（Betaflight / ArduPilot / PX4 三方交叉验证）、
Betaflight 架构导读、从零写路线图。

**未决 —— 需要拿板子实测**（详见 `docs/01` §6）：

- [ ] 备份出厂 Flash（**先做这个**）
- [ ] IMU 型号（四个来源说法不一，读 `WHO_AM_I`）
- [ ] 气压计确认（三方指向 SPL06，BF config 缺这行）
- [ ] IMU 朝向（三方定义两两不一致）
- [ ] 电压/电流标定（BF `120` vs AP/PX4 `83.3` 冲突）
- [ ] CAN 焊盘是否引出
- [ ] Flash 分区（bootloader 占多少）

## 三个关键提醒

1. **本地 `reference/H743飞控/` 是 Matek H743-WING 复刻，引脚不能套到大科板子上**
2. **Betaflight master 的编译命令是 `make CONFIG=DAKEFPVH743PRO`**，
   不是老教程里的 `make TARGET=...`；且必须用 GCC，Keil 编不了
3. **`PA15` 是 JTDI 引脚**（作 Flash 片选），调试**必须用 SWD 不能用 JTAG**
