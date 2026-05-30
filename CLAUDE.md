# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

STM32F103C8T6 (Blue Pill) 的 SimpleFOC 电机控制项目。CubeMX 6.17.0 生成，使用 STM32Cube FW_F1 V1.8.7，CMake + Ninja + arm-none-eabi-gcc 构建。

## 构建命令

```bash
# 配置 (Debug)
cmake --preset Debug

# 编译
cmake --build --preset Debug

# 或指定 build 目录
cmake --build build/Debug

# 清理
cmake --build build/Debug --target clean

# Release 构建
cmake --preset Release
cmake --build --preset Release
```

VSCode 中使用 `cube-cmake` 包装器（STM32CubeIDE 自带），预设已配好 toolchain 路径。

## 硬件配置

| 外设 | 引脚 | 用途 |
|------|------|------|
| TIM1 CH1/CH2/CH3 | PA8/PA9/PA10 | 三相 PWM 输出（BLDC 驱动） |
| TIM2 (IRQ) | — | 控制循环/电流采样定时器 |
| I2C1 SCL/SDA | PB8/PB9 | 磁编码器（如 AS5600） |
| USART1 TX/RX | PB6/PB7 | 115200 8N1，串口调试 |
| GPIO OUT | PA11 | 通用输出 |
| SWD | PA13/PA14 | 调试接口 |

系统时钟：HSE 8MHz → PLL ×9 → 72MHz SYSCLK。APB1 36MHz，APB2 72MHz。

## 代码架构

```
simpleFOC_1/
├── Core/                    # 用户代码（CubeMX 只在 USER CODE 区域写，不覆盖其余）
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h   # HAL 模块开关
│   │   └── stm32f1xx_it.h
│   └── Src/
│       ├── main.c                 # 入口，外设初始化后进入空 while(1)
│       ├── stm32f1xx_hal_msp.c    # MCU 引脚映射和时钟使能
│       ├── stm32f1xx_it.c         # 中断服务：TIM2_IRQ, USART1_IRQ
│       ├── system_stm32f1xx.c     # CMSIS SystemInit / SystemCoreClockUpdate
│       ├── sysmem.c               # _sbrk() 堆实现
│       └── syscalls.c             # POSIX syscall 桩（_write 等）
├── Drivers/                 # HAL 库和 CMSIS（只读，不修改）
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/     # 编译为 STM32_Drivers 对象库
├── cmake/
│   ├── gcc-arm-none-eabi.cmake   # 工具链：arm-none-eabi-gcc, cortex-m3, nano.specs
│   └── stm32cubemx/CMakeLists.txt # 驱动层 CMake（sources, includes, defines）
├── CMakeLists.txt           # 顶层 CMake（在此添加用户源文件和库）
├── CMakePresets.json        # Debug/Release 预设，Ninja 生成器
├── STM32F103XX_FLASH.ld     # 链接脚本：64K FLASH, 20K RAM, 512B 堆, 1K 栈
├── startup_stm32f103xb.s    # 启动汇编（向量表，Reset_Handler）
└── simpleFOC_1.ioc          # CubeMX 硬件配置
```

## 开发要点

- **添加用户源文件**：修改顶层 `CMakeLists.txt`，在 `target_sources()`、`target_include_directories()` 等处添加。
- **添加 SimpleFOC 库**：建议将 SimpleFOC 源码放入新目录，在 `CMakeLists.txt` 中 `add_subdirectory()` 或直接 `target_sources()` 加入。SimpleFOC 依赖 Arduino API 层，需要 HAL 适配层（GPIO、PWM、SPI/I2C、时间函数）。
- **CubeMX 代码生成规则**：CubeMX 只在 `/* USER CODE BEGIN */ ... /* USER CODE END */` 区域内保留用户代码，其他区域会被重新生成覆盖。所有自定义逻辑写在 USER CODE 区域内或独立文件中。
- **编译定义**：`USE_HAL_DRIVER`、`STM32F103xB`（在 cmake/stm32cubemx/CMakeLists.txt 中设置）。
- **中断优先级**：TIM2_IRQ 优先级 0，USART1_IRQ 优先级 1，SysTick 优先级 15。
- **工具链版本**：GCC 14.3.1+st.2，CMake 4.3.1+st.1，Ninja 1.13.2+st.1（ST 发行版）。
- **无测试框架**：项目当前无单元测试或 CTest 配置。

## 外设句柄

全局句柄在 `main.c` 中定义：
- `I2C_HandleTypeDef hi2c1`
- `TIM_HandleTypeDef htim1`（PWM）
- `TIM_HandleTypeDef htim2`（Base + 中断）
- `UART_HandleTypeDef huart1`
