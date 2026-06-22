# FOME (Free Open Motorsports ECU) 源码分析文档

## 文档结构

| 文档 | 说明 |
|------|------|
| [core_analysis.md](core_analysis.md) | 主干核心代码分析 |
| [ecu_architecture.md](ecu_architecture.md) | ECU 架构文档 |
| [sensors.md](sensors.md) | 传感器子系统文档 |
| [actuators.md](actuators.md) | 执行器子系统文档 |
| [scenarios/](scenarios/) | 汽车控制场景分析 |
| [scenarios/cranking.md](scenarios/cranking.md) | 起动场景 |
| [scenarios/idle_control.md](scenarios/idle_control.md) | 怠速控制场景 |
| [scenarios/fuel_control.md](scenarios/fuel_control.md) | 燃油控制场景 |
| [scenarios/ignition_timing.md](scenarios/ignition_timing.md) | 点火正时场景 |
| [scenarios/electronic_throttle.md](scenarios/electronic_throttle.md) | 电子节气门场景 |
| [scenarios/boost_control.md](scenarios/boost_control.md) | 增压控制场景 |
| [scenarios/vvt_control.md](scenarios/vvt_control.md) | VVT 控制场景 |

## 项目概述

FOME (Free Open Motorsports ECU) 是一个基于 STM32 微控制器的开源发动机控制单元固件，源自 rusEFI 项目，专注于用户体验和稳定性。

### 核心架构原则

- **事件驱动执行**: 曲轴/凸轮轴传感器的触发事件驱动主控制循环
- **角度调度**: 燃油喷射和点火事件按曲轴角度调度，而非仅按时间
- **配置驱动**: 主板和发动机参数外部化，固件通过配置适应不同硬件
- **ChibiOS RTOS**: 实时操作系统基础

### 代码目录结构

```
firmware/
  controllers/       # 核心控制逻辑
    algo/            # 燃油/点火/进气计算
    actuators/       # 执行器控制
    engine_cycle/    # 发动机同步输出
    sensors/         # 传感器输入处理
    trigger/         # 曲轴/凸轮位置解码
    can/             # CAN 总线通信
    lua/             # 运行时脚本
  hw_layer/          # 硬件抽象层
  config/boards/     # 主板配置
  config/engines/    # 发动机配置
```
