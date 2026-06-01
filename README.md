# IDC Robocon 2026 — 浙江大学内部选拔赛

本项目包含了 **Nathan 和 HXC** 参加 **2026 IDC Robocon 浙江大学内部选拔赛**的完整机器人控制源码与配套工程资源。

本次校内选拔赛的赛题由 **IDC Robocon 2025** 的官方比赛规则改编而来。我们的机器人在此规则框架下完成了机构设计与控制系统开发。详细积分规则、场地尺寸与任务说明见仓库目录下的 **`IDC 2026 校赛规则（新）.pdf`**。

> ⚠️ 项目目前仍在施工中。

---

## 目录结构

```
├── README.md                          # 本文件
├── IDC 2026 校赛规则（新）.pdf           # 校内选拔赛规则文档
├── .gitignore                         # Git 忽略规则（排除 Keil/MDK 编译产物）
├── solidworks/                        # 机械结构 SolidWorks 设计文件
│   ├── 底板.SLDPRT                     # 底盘底板
│   ├── 中盘.SLDPRT                     # 中层结构板
│   ├── 叶.SLDPRT                       # 功能叶片/桨叶
│   ├── gear1.SLDPRT                   # 齿轮零件
│   ├── shang1.SLDPRT                  # 上端零件
│   ├── 电机架.SLDPRT                   # M2006 电机固定架
│   ├── TB6600架.SLDPRT                # TB6600 步进驱动器固定架
│   ├── 概念版.SLDASM                   # 概念版整机装配体
│   └── 装配体5.SLDASM                  # 版本5整机装配体
└── 程序源码/                           # STM32 嵌入式固件工程
    ├── demo.ioc                       # STM32CubeMX 项目配置文件
    ├── keilkilll.bat                  # Keil 编译临时文件清理脚本
    ├── M2006&C610 J-Flash Project.jflash  # J-Flash 烧录配置
    ├── Drivers/                       # HAL 库 & CMSIS 驱动（STM32Cube FW_F4 V1.18.0）
    │   ├── CMSIS/                     # ARM CMSIS 核心 & DSP 库
    │   └── STM32F4xx_HAL_Driver/      # STM32F4 HAL 外设驱动
    ├── Inc/                           # 用户头文件
    │   ├── main.h                     # 主头文件 & 引脚宏定义
    │   ├── can.h                      # CAN 外设句柄声明
    │   ├── dma.h / gpio.h / tim.h / usart.h  # 各外设初始化声明
    │   ├── ir8.h                      # 红外（备用）
    │   ├── Remote_Control.h           # DJI 遥控器 DBUS 协议结构体
    │   └── stm32f4xx_hal_conf.h       # HAL 模块配置
    ├── Src/                           # 用户源文件
    │   ├── main.c                     # ★ 主控逻辑（运动控制、舵机、步进、PS2）
    │   ├── can.c                      # CAN1 初始化 & 错误处理
    │   ├── dma.c / gpio.c / tim.c / usart.c  # 外设初始化
    │   ├── ir8.c                      # 红外（备用）
    │   ├── Remote_Control.c           # DJI 遥控器 DBUS 解析
    │   ├── stm32f4xx_hal_msp.c        # HAL 底层 MSP 初始化
    │   ├── stm32f4xx_hal_timebase_TIM.c  # FreeRTOS/HAL 时基（TIM6）
    │   ├── stm32f4xx_it.c             # 中断服务函数
    │   └── system_stm32f4xx.c         # 系统时钟启动代码
    ├── MDK-ARM/                       # Keil MDK-ARM 工程文件
    │   ├── demo.uvprojx               # Keil 工程文件
    │   ├── startup_stm32f427xx.s      # 启动汇编（STM32F427xx）
    │   └── bsp/                       # ★ 板级支持包
    │       ├── bsp_can.{c,h}          # M2006/C610 CAN 总线电机驱动
    │       ├── pid.{c,h}              # PID 控制器（位置式 / 速度式）
    │       ├── ps2.{c,h}              # PS2 无线手柄通讯协议
    │       ├── ir8.{c,h}              # 红外传感器（备用）
    │       └── mytype.h               # 通用类型定义
    └── Middlewares/                   # 中间件
        └── Third_Party/FreeRTOS/      # FreeRTOS V9.0（已集成，主循环未使用）
```

---

## 硬件平台

| 组件           | 型号 / 规格                    | 说明                          |
| -------------- | ------------------------------ | ----------------------------- |
| 主控           | STM32F427IIHx (Cortex-M4)     | 168MHz, 256KB RAM, 2MB Flash |
| 驱动电机 ×2    | M2006 + C610 电调              | CAN 总线通讯, 8,000mA 峰值    |
| 步进电机       | 42/57 步进 + TB6600 驱动器     | 脉冲+方向控制, 最高 10.5k step/s |
| 舵机 ×2        | 360° 连续旋转舵机 + 180° 角度舵机 | PWM 控制 (TIM4 CH3/CH4)   |
| 继电器         | 电磁铁/继电器模块              | GPIO 控制, PWM 脉冲驱动       |
| 遥控器         | PS2 无线手柄                   | USART1, 100kbps, Even Parity |
| 调试接口       | SWD (PA13/PA14)               | J-Link / ST-Link              |
| 板载 LED ×2    | PF14 (LED1) / PE7 (LED2)      | 状态指示 & 调试               |

### MCU 时钟树

- **HSE**: 12MHz 外部晶振
- **PLL**: ×168 / ÷6 = 336MHz VCO → ÷2 = **168MHz SYSCLK**
- **APB1**: 42MHz, **APB2**: 84MHz, **CAN**: 1Mbps (42MHz / 3 / 14)

### CAN 总线

- 波特率：**1 Mbps** (Prescaler=3, BS1=9, BS2=4)
- CAN ID 分配：`0x200` 广播, `0x201` 电机1, `0x203` 电机2
- 支持 Auto Bus-Off Recovery (`ABOM`)
- TX mailbox 卡死自动释放的错误恢复机制

---

## 控制方案

### 1. 底盘运动控制

- **驱动方式**: 双轮差速驱动（differential drive）
- **速度闭环**: PID 速度控制器
  - 正常模式: `Kp=1.6, Ki=0.25, Kd=0.70`, 积分限幅 `4000`
  - 爬坡模式 (CIR键触发): `Kp=3.0, Ki=1.2, Kd=0.50`, 积分限幅 `6500`
- **Slew Rate 限幅**: 200 RPM/10ms，防止急加速失步
- **目标转速范围**: ±4000 RPM（前进/后退）、±3000 RPM（转向）
- **启动保护**: 上电后 500ms 内不输出电机电流

### 2. PS2 遥控器映射

| 按键/摇杆      | 功能                        |
| -------------- | --------------------------- |
| 左摇杆 Y       | 前进 / 后退                 |
| 右摇杆 X       | 左转 / 右转                 |
| L1             | 速度 ×0.8（减速）           |
| R1             | 速度 ×1.25（加速）          |
| ○ (CIR)        | 4 秒自动爬坡 (2500 RPM)     |
| L2             | 180° 舵机 收缩              |
| R2             | 180° 舵机 伸展              |
| D-Pad ↑        | 步进电机 上升               |
| D-Pad ↓        | 步进电机 下降               |
| D-Pad ←        | 360° 舵机 逆时针旋转        |
| D-Pad →        | 360° 舵机 顺时针旋转        |

- **摇杆校准**: 上电自动记录摇杆中位（calibration），消除静态偏移
- **死区**: ±20（摇杆原始值 ±127 范围内忽略）
- **按键消抖**: 3-5 帧（30-50ms）防抖确认；长按 10 帧确认并加速
- **断连保护**: 连续 10 帧（100ms）丢失信号判定为断连 → 自动停止所有电机并尝试重连，重连需 30 帧（300ms）稳定

### 3. 自动爬坡功能 (CIR 按键)

按下 ○ 键触发 4 秒自动前进行程：
- 目标转速：**2500 RPM**
- PID 切换为 **爬坡专用参数**（高积分以对抗重力下滑）
- **Cross-Couple 轮速同步 PI 控制器**：
  - `Sync P=0.35, Sync I=0.08, 积分限幅±250 RPM, 总修正限幅±400 RPM`
  - 检测左右轮速差，前馈修正防止坡道偏航
- 结束后平滑减速并切回正常 PID 参数
- 爬坡期间步进电机输出停止

### 4. 步进电机控制

- **驱动方式**: 定时器 PWM 脉冲 + 方向引脚 (PB0=PUL, PB1=DIR)
- **控制**: D-Pad ↑/↓，加速斜坡：每 10ms 减 280 到最低 2000（对应 ~21k step/s）
- **方向切换**: 改变方向时自动停止当前脉冲、复位计数器再启动

### 5. 舵机控制

| 通道    | 舵机类型     | 控制按键       | 行为说明                                      |
| ------- | ------------ | -------------- | --------------------------------------------- |
| Servo0  | 360° 连续    | D-Pad ←/→     | 1500μs=停止, 速度分两档(150/350μs 偏移)       |
| Servo1  | 180° 角度    | L2 / R2        | 1360-1735μs, 行程锁定（单次最多±150μs 偏移）  |

---

## 编译与烧录

### 环境要求

- **IDE**: Keil MDK-ARM V5.0+
- **STM32Cube FW**: STM32Cube FW_F4 V1.18.0
- **烧录工具**: J-Link / ST-Link / J-Flash

### 编译步骤

1. 用 Keil MDK 打开 `程序源码/MDK-ARM/demo.uvprojx`
2. 确认 MCU 型号为 STM32F427IIHx
3. 编译 (`F7`)，产物在 `程序源码/MDK-ARM/demo/` 目录：
   - `demo.axf` — 调试文件
   - `demo.hex` — 烧录文件（35KB）

### 清理编译文件

运行 `程序源码/keilkilll.bat` 删除所有 `.o`, `.crf`, `.axf`, `.map`, `.dep` 等中间产物。

---

## 调试

### 启动 LED 诊断

系统上电后通过 **LED1 (PF14)** 指示灯判断启动阶段：

| LED1 状态      | 诊断信息                          |
| -------------- | --------------------------------- |
| 灭             | 未上电或硬件故障                  |
| 常亮           | 卡在 `HAL_Init()`                 |
| 亮→灭→亮       | 卡在 `SystemClock_Config()` / HSE |
| 1Hz 闪烁       | ✅ 正常进入主循环                 |

### 运行时变量监控

可在调试器中观察以下关键变量：
- `moto_chassis[0/1].speed_rpm` — 左右电机实测转速
- `motor_pid[0/1].target` — 左右轮目标转速
- `actual_target_l / actual_target_r` — 经 slew rate 平滑后的目标值
- `ps2_connected` — PS2 手柄连接状态
- `ps2_ok_cnt / ps2_fail_cnt` — PS2 通信成功/失败计数
- `can_err_cnt / can_tx_fail_cnt` — CAN 总线错误/发送失败计数

---

## 已知问题 / TODO

- [ ] 遥控距离在比赛场地可能存在干扰
- [ ] 爬坡参数需在真实 30° 坡道上测试调优
- [ ] 步进电机急停 / 堵转保护未实现
- [ ] FreeRTOS 已集成但未启用多任务调度，目前仅使用裸机主循环
- [ ] 自动阶段（非遥控）策略尚待开发

---

## 参考资料

- [IDC Robocon 2025 官方规则](https://idcrobocon.org/)
- [STM32F427 Reference Manual (RM0090)](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [M2006 电机手册 (DJI RoboMaster)](https://www.robomaster.com/)
- PS2 手柄 SPI 协议 — 详见 `MDK-ARM/bsp/ps2.c`

---

## 作者

- **Nathan** — 嵌入式控制 & 软件架构
- **HXC** — 机械结构设计 & 硬件搭建

*Zhejiang University, 2026*
