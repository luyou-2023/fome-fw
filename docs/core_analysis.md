# 主干核心代码分析

## 1. 系统启动流程 (`firmware/rusefi.cpp`)

### 启动序列

```
runRusEfi()
  ├── engine->setConfig()                    # 清除并设置配置
  ├── startLoggingProcessor()                # 启动日志系统
  ├── checkLastBootError()                   # 检查上次启动错误
  ├── initDataStructures()                   # 初始化表格对象
  ├── initHardwareNoConfig()                 # 最小硬件初始化(GPIO等)
  ├── detectBoardType()                      # 识别主板型号
  ├── engineModules.initNoConfiguration()    # 初始化发动机模块
  ├── startUsbConsole() / initUsbMsd()       # USB通信
  ├── loadConfiguration()                    # 从Flash读取配置
  ├── startTunerStudioConnectivity()         # 启动TS通信
  ├── runRusEfiWithConfig()                  # 完整初始化(含配置)
  │     ├── setjmp()                         # 断言失败恢复点
  │     ├── startStatusThreads()             # LED闪烁等
  │     ├── initHardware()                   # ADC/PWM/CAN等
  │     ├── startLua()                       # Lua脚本引擎
  │     └── initEngineController()           # 发动机控制器初始化
  ├── initMainLoop()                         # 启动1000Hz主循环
  ├── updateBootloader()                     # 检查/更新引导程序
  └── runMainLoop()                          # 空闲主循环(200ms休眠)
```

### 关键设计决策

- **`setjmp`/`longjmp` 恢复机制**: 初始化期间如果断言失败，可以通过 `longjmp` 回到 `runRusEfiWithConfig()` 的 `setjmp` 恢复点，避免完全死机。这是嵌入式系统中轻量级的异常恢复手段，不像C++异常那样需要RTTI支持。
- **分阶段硬件初始化**: `initHardwareNoConfig()` 先做基本GPIO初始化（保证基本通信能力），`initHardware()` 再做完整硬件初始化（ADC、PWM、CAN等依赖配置的部分）。这种分层确保即使配置损坏，也能通过USB刷写新配置恢复。

---

## 2. 主循环 (`firmware/controllers/core/main_loop.cpp`)

### 时序架构

主循环以 **1000 Hz** 运行，使用周期计数器将不同频率的任务错开执行：

| 频率 | 周期 | 运行内容 |
|------|------|----------|
| 500 Hz | 2ms | `updateSlowAdc()` + 电子节气门更新 |
| 250 Hz | 4ms | `engine->periodicFastCallback()` (燃油/点火计算) |
| 20 Hz | 50ms | `doPeriodicSlowCallback()` (传感器/看门狗) |

### 周期调度算法

```c
LoopPeriod MainLoop::makePeriodFlags() {
    // 使用 m_cycleCounter 从0计数到999，然后归零
    // 通过模运算确定当前周期需要执行哪些任务
    // 例如: 500Hz 每2个周期执行一次(1000/500=2)
    //      250Hz 每4个周期执行一次(1000/250=4)
    //      20Hz  每50个周期执行一次(1000/20=50)
}
```

**为什么这样设计**: 将不同频率的任务合并到单个线程中，通过简单的模运算分时调度。避免了多个定时器线程的复杂同步和资源竞争，同时保持了确定性的执行时序。`m_cycleCounter` 溢出到1000时归零，防止整数溢出。

### Stall检测

```c
if (m_stallTimer.hasElapsedSec(0.1)) {
    // 如果主循环超过100ms没有运行，打印警告
    // 这通常意味着高优先级任务(如中断)占用了太多CPU时间
}
```

---

## 3. 触发系统 (`firmware/controllers/trigger/`)

### 架构

```
硬件中断 → hwHandleShaftSignal()
              → handleShaftSignal()         # 信号索引→TriggerEvent
                → TriggerCentral::handleShaftSignal()
                    → isToothExpectedNow()    # 噪声滤波
                    → decodeTriggerEvent()    # 解码触发事件
                    → rpm计算
                    → mainTriggerCallback()   # 燃油/火花调度
```

### 触发解码器 (`trigger_decoder.cpp`)

`decodeTriggerEvent()` 是核心解码函数：

1. **间隙比同步**: 对于需要同步的触发轮，计算连续齿之间的时间间隔之比，与预定义的同步间隙比比较。如果匹配，则完成同步。
2. **无需同步**: 对于某些触发模式，同步点是指数达到周期末尾时。
3. **验证**: 在同步点验证事件计数是否符合预期。

**为什么间隙比而不用绝对时间**: 间隙比与转速无关——无论发动机以1000 RPM还是7000 RPM运转，同步齿的间隙与常规齿的间隙之比保持不变。这使解码器完全不受转速影响。

### 噪声抑制 (`isToothExpectedNow`)

```c
// 两个阈值:
// 1. 短于0.5°的齿间隔 → 加倍边沿(电气噪声)
// 2. 在1000RPM以上误差>10° → 无效信号
// 这样可以滤除绝大多数电气干扰
```

### VVT解码

凸轮轴信号独立于曲轴信号处理，通过测量凸轮齿相对于曲轴同步点的角度位置来计算VVT位置。支持多品牌VVT模式（丰田3齿等）。

---

## 4. 发动机状态管理 (`firmware/controllers/algo/engine.cpp`)

### Engine 类

`Engine` 是整个固件的"上帝对象"，聚合了所有子系统和状态：

- **`periodicFastCallback()`** (250Hz): 执行所有发动机模块的快速回调，包括 `engineState.periodicFastCallback()` → 燃油/点火计算
- **`periodicSlowCallback()`** (20Hz): 重新读取触发配置、检查同步丢失、看门狗、慢速传感器、TPS加速补偿、VR PWM、氧传感器加热器、起动继电器等

### 同步丢失恢复

```c
void Engine::OnTriggerSynchronizationLost() {
    // 1. 重置RPM计算器
    // 2. 重置触发状态和瞬时RPM
    // 3. 重置VVT状态
    // 4. 使所有喷射和点火调度无效
    // 5. 通知所有发动机模块
}
```

**为什么需要使所有调度无效**: 如果曲轴相位丢失（例如在减速时触发信号噪声），正在等待执行的燃油喷射和点火事件可能基于错误的曲轴位置。必须取消它们，避免在错误的相位喷油或点火，可能导致发动机回火或损坏。

### ECU看门狗

```c
void Engine::efiWatchdog() {
    // 如果发动机最近没有转动
    // → 关闭喷油器和点火
    // → 进入安全状态
}
```

---

## 5. 触发回调主干 (`firmware/controllers/engine_cycle/main_trigger_callback.cpp`)

每次触发齿事件发生时触发：

```c
mainTriggerCallback(trgEventIndex, phase)
  ├── if trgEventIndex == 0:            # 同步点
  │     ├── 检查触发配置是否变化
  │     └── 使调度无效(如果需要)
  ├── 所有发动机模块的 onEnginePhase()
  ├── handleFuel()                       # 喷射事件调度
  └── onTriggerEventSparkLogic()         # 点火事件调度
```

**为什么在同步点检查配置变更**: `trgEventIndex == 0` 是发动机周期的起始点（通常是压缩上止点前）。在这里检查配置变化并重建调度，确保整个周期使用一致的配置数据，不会在周期中间发生配置变化。

---

## 6. 燃油计算 (`firmware/controllers/algo/fuel_math.cpp`)

### 空燃比控制算法

```
getCycleInjectionMass()
  ├── 选择空气量模型 (速度密度/MAF/Alpha-N/Lua)
  ├── 计算气缸空气量
  ├── 空气量 → 目标空燃比 → 燃油质量
  ├── 全局燃油修正
  ├── 起动燃油修正 (起动转数衰减 + 水温 + TPS)
  ├── 运行修正 (IAT/CLT/气压/起动后/ALS/弹射)
  ├── DFCO (减速断油)
  └── TPS加速补偿
```

### 起动燃油算法

```c
getCrankingFuel3(baseFuel, revolutionCounterSinceStart)
  // 三阶段修正:
  // 1. 转数衰减系数: 随着发动机转动次数增加，燃油逐渐减少
  //    (因为气缸壁的燃油膜已经建立)
  // 2. 水温系数: 冷起动需要更多燃油
  //    (低温时燃油蒸发不良，需要浓混合气)
  // 3. TPS系数: 如果踩油门起动，增加燃油
  //    (清缸模式 / 油门辅助)
```

### 加速补偿

TPS加速补偿检测TPS变化率（即节气门打开速度），当快速踩下油门时额外增加燃油，补偿进入气缸的空气突然增多而燃油因壁膜延迟跟不上的问题。

**算法**: 基于TPS变化率查询2D或3D表，获得额外的燃油质量。这个额外燃油也会经过壁膜模型(`wall_fuel.cpp`)处理，模拟燃油在进气道壁面的沉积和蒸发过程。

---

## 7. 点火控制 (`firmware/controllers/engine_cycle/spark_logic.cpp`)

### 点火调度流程

```
onTriggerEventSparkLogic()
  ├── 准备点火调度 (首次或配置变更时)
  │     ├── 计算每个气缸的最佳点火提前角
  │     ├── 计算 dwell (线圈充电) 角度
  │     └── 验证 dwell 角度是否超过周期限制
  ├── 对每个气缸:
  │     ├── 检查 dwell 起始角是否在当前触发相位窗口内
  │     ├── 应用点火限制 (弹射/扭矩降低/ALS)
  │     └── 调度 dwell 开始和火花触发
  └── 火花触发回调:
        ├── 关闭线圈 (产生高压火花)
        ├── 可选多火花 (稀薄燃烧时)
        └── 可选 trailing spark (高能点火)
```

### Dwell 保护

**Underdwell保护**: 如果转速突然升高导致实际充电时间(dwell)不足，自动推迟点火提前角（提前角减小），让线圈有更多时间充电，确保火花能量。

**Overdwell保护**: 如果点火因弹射限制等原因被跳过，而线圈已经开始充电，会安排一个"保护火花"来释放线圈能量，防止线圈过热损坏。

### 点火模式

- 独立点火 (COP): 每个气缸一个线圈，最灵活
- 双缸点火 (Wasted Spark): 每两个气缸共享一个线圈，气缸对同时在压缩和排气上止点点火
- 单线圈(分电器): 一个线圈通过机械分电器分配给所有气缸

---

## 8. 调度器

调度器(`Scheduler`)是这个固件的核心定时服务。策略模式实现：

- **生产固件** (`EFI_PROD_CODE`): `SingleTimerExecutor` - 基于单个硬件定时器的调度器，所有角度和时间事件都注册到同一个定时器。
- **模拟器** (`EFI_SIMULATOR`): `SleepExecutor` - 基于睡眠的调度器。
- **测试** (`EFI_UNIT_TEST`): `TestExecutor` - 用于单元测试。

所有燃油喷射和点火事件都通过调度器在精确的曲轴角度触发。调度器将角度转换为时间（基于当前RPM），然后设置硬件定时器在指定时间后执行回调。
