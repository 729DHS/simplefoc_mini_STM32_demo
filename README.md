# SimpleFOC Mini — STM32F103 位置伺服 Demo

> 基于 STM32F103C8T6 (Blue Pill) + SimpleFOC Mini 驱动板 + AS5600 磁编码器 + 2804 云台电机  
> CubeMX 6.17.0 / HAL / CMake + Ninja / arm-none-eabi-gcc 14.3

## 硬件

| 外设 | 引脚 | 说明 |
|------|------|------|
| PWM (TIM1 CH1/2/3) | PA8 / PA9 / PA10 | SimpleFOC Mini IN1/IN2/IN3 |
| ENABLE | PA11 | SimpleFOC Mini EN 引脚 |
| AS5600 I2C | PB8 (SCL) / PB9 (SDA) | **软件 I2C**, 非硬件外设 |
| USART1 | PB6 (TX) / PB7 (RX) | 115200 8N1 串口 |
| SWD | PA13 / PA14 | 调试/烧录 |
| 电机 | 2804 云台电机 | 12N14P (7 极对) |

## 构建 & 烧录

### 编译

```bash
cmake --preset Debug
cmake --build --preset Debug
```

### 烧录 (ST-LINK / cmsis-dap)

```bash
openocd -s <scripts_dir> -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program build/Debug/simpleFOC_1.elf verify reset exit"
```

> 注意：PATH 需包含 `arm-none-eabi-gcc`（STM32CubeIDE 自带, 位于 `~/.stm32cube/bundles/gnu-tools-for-stm32/`）

## 串口命令

波特率 **115200 8N1**, 换行结束, 大小写不敏感。

### 模式控制

| 命令 | 示例 | 说明 |
|------|------|------|
| `M0` / `M1` | `M1` | 开环速度 / 位置伺服 |
| `E1` / `E0` | `E1` | 使能 / 禁用电机 |
| `T<deg>` | `T90` `T-45` | 目标角度（伺服模式, 自动找最近实例） |
| `V<0~1>` | `V0.3` | 电压限幅 (PWM 占空比) |
| `S<rpm>` | `S60` | 开环转速 (速度模式) |
| `P<n>` | `P7` | 极对数 |

### PID 调参

| 命令 | 示例 | 推荐范围 |
|------|------|---------|
| `Kp<n>` | `Kp0.1` | 0.03 ~ 0.5 |
| `Ki<n>` | `Ki0.02` | 0 ~ 0.1 |
| `Kd<n>` | `Kd0.01` | 0 ~ 0.05 |

> 调参自动重置积分, 无瞬态冲击。  
> Kd >0.02 时 AS5600 量化噪声引发 limit cycle, 慎用。

### 诊断

| 命令 | 说明 |
|------|------|
| `D` | 编码器诊断: 磁铁检测 + 5 次原始读数 |
| `?` | 完整状态: 角度/PID/极对数/I2C 计数 |
| `C` | 切换自动文本打印 (0.5s 间隔) |
| `O1` / `O0` | VOFA+ FireWater 输出 (110Hz 角度曲线) |
| `H` | 帮助 |

### 自动状态行格式

```
> A:1170 deg:102.8 err: +3.5 tgt:107.2 v:0.40 [SERVO] i2c:0
```

| 字段 | 含义 |
|------|------|
| `A` | AS5600 原始值 (0~4095) |
| `deg` | 机械角度 (°) |
| `err` | 累计角度误差 (°), 0=到达, ±360=多转一圈 |
| `tgt` | 目标角度 (°) |
| `v` | 电压限幅 |
| `[SERVO/SPEED/OFF]` | 当前模式 |
| `i2c` | I2C 连续失败计数 |

## VOFA+ 可视化

```
O1                  ← 开启 FireWater 输出
```

VOFA+ 中选 **FireWater** 协议, 通道数=4：
1. 机械角度 (°)
2. 目标角度 (°)
3. 位置误差 (°)
4. I2C 错误计数

110Hz 刷新, 可实时观察阶跃响应和 PID 收敛曲线。

## 代码架构

```
Core/
├── Inc/
│   ├── as5600.h          # AS5600 驱动 (软件 I2C 接口)
│   ├── foc.h              # FOC 电机控制 (双模式)
│   ├── pid.h              # PID 控制器 (D-on-measurement + 低通滤波)
│   ├── soft_i2c.h         # 软件 GPIO I2C (PB8/PB9)
│   └── stm32f1xx_hal_conf.h
├── Src/
│   ├── main.c             # 主循环 / 串口命令 / VOFA / 传感器读取
│   ├── as5600.c
│   ├── foc.c              # 电机控制核心
│   ├── pid.c              # PID 实现 (条件积分 + 累计角度)
│   ├── soft_i2c.c         # 位操作 I2C START/STOP/ACK/READ/WRITE
│   └── stm32f1xx_it.c     # 中断服务 + HardFault 诊断
├── Drivers/               # STM32CubeMX HAL (只读)
├── cmake/                 # 工具链 & 构建配置
├── docs/
│   └── debugging-journal.md  # 7 个硬件/算法/C 语言 bug 全记录
├── analysis/
│   └── pid_sim.m          # MATLAB PID 量化噪声仿真
├── STM32F103XX_FLASH.ld   # 链接脚本 (栈 2KB)
└── CMakeLists.txt
```

## 关键技术细节

- **软件 I2C**：GPIO 位操作, 避开 STM32F103 硬件 I2C 的 BUSY 卡死 bug
- **累计角度**：AS5600 绝对值 0~360° → unwrap 为无跳变累计角度, PID 跨圈运行
- **D-on-measurement + 低通滤波**：微分项只反映实际速度, setpoint 跳变不产生尖峰
- **IWDG 看门狗**：主循环卡死 >1 秒自动复位
- **HardFault 诊断**：崩溃时串口输出 PC/LR/CFSR/HFSR

## 完整排错记录

见 [`docs/debugging-journal.md`](docs/debugging-journal.md)——从 I2C 卡死到 C 语言整型提升, 7 个硬 bug 的完整排查链。
