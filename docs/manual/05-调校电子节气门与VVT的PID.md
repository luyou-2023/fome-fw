# 第五章：调校电子节气门与 VVT 的 PID

## 5.1 PID 控制基础

标准 PID 公式：
```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt
```

其中：
- **Kp** (比例): 响应当前误差，Kp 越大响应越快，但过大会振荡
- **Ki** (积分): 消除稳态误差，Ki 过大会导致积分饱和和超调
- **Kd** (微分): 预测误差趋势，抑制振荡，但对噪声敏感

更多内容参考 [Wikipedia PID Controller](https://en.wikipedia.org/wiki/PID_controller#Fundamental_operation)

## 5.2 电子节气门 (ETB) PID 调校

### 控制架构

```
踏板位置(冗余传感器)
  ↓
目标位置计算 (pedalToTpsTable 3D表)
  ↓
开环前馈 (etbBiasBins/etbBiasValues) ---→ 占空比 ←--- PID闭环
                                                ↓
                                           H桥电机
                                                ↓
                                         节气门位置(冗余TPS)
```

### 自整定功能

FOME 支持 Åström–Hägglund 继电反馈自整定：
1. 在目标位置附近做 ±5% 的 bang-bang 振荡
2. 测量极限增益 Ku 和极限周期 Tu
3. 用 Ziegler-Nichols 规则计算 PID 参数：
   - Kp = 0.6 * Ku
   - Ki = Kp / (0.5 * Tu)
   - Kd = Kp / (0.125 * Tu)

### 前馈（Bias）表

前馈表（etbBiasBins/etbBiasValues）补偿节气门的弹簧力和气动力：
- 低开度：气流速度高，吸力大（文丘里效应），需要更多占空比
- 中开度：弹簧力与气动力接近平衡，所需占空比最小
- 大开度：弹簧力主导，需要更多占空比

**代码处理流程：**

`firmware/controllers/actuators/electronic_throttle.cpp` — 电子节气门控制核心：
```c
EtbController::update()  // 每 2ms (500Hz) 执行
  → checkStatus()        // 传感器健康检查，冗余校验
  → getSetpoint()        // 计算目标位置
    → getSetpointEtb()   // 踏板映射 + 扭矩模型 + 限速
  → getOpenLoop()        // 前馈 (etbBiasBins表)
  → getClosedLoop()      // PID 反馈
    → getClosedLoopAutotune()  // 自整定模式
  → setOutput()          // 设置电机占空比 [钳位 ±0.9]
```

PID 自整定算法 (`electronic_throttle.cpp`):
```c
getClosedLoopAutotune(target, actual):
  // 1. 计算误差 e = target - actual
  // 2. 如果 |e| > 阈值 → bang-bang 控制
  // 3. 测量振荡幅度 A 和周期 Tu
  // 4. 计算: Ku = 4d/(πA), d = 振荡幅值
  // 5. Ziegler-Nichols: Kp=0.6Ku, Ki=Kp/(0.5Tu), Kd=Kp/(0.125Tu)
```

安全机制：
```c
// 冗余传感器校验: TPS1 和 TPS2 读数必须成比例
// 连续 50 次校验失败 → 进入跛行模式
// 跛行模式: 断开电机 → 弹簧回到 15% 开度
```

## 5.3 VVT PID 调校

### 控制架构

```
目标角度 (3D表: RPM × 负荷)
  ↓
PID 控制器 → PWM → OCV 机油控制阀 → 凸轮轴相位器
                                                ↑
凸轮轴位置传感器 (曲轴参考角度差)
```

**代码处理流程：**

`firmware/controllers/actuators/vvt.cpp` — VVT 控制核心：
```c
VvtController::onFastCallback()  // 250Hz
  → 检查 RPM 是否达到控制最低转速
  → 检查 CLT 是否够暖
  → 检查发动机运行时间 > 激活延迟
  → update() 闭环控制
```

VVT 角度反馈检测 (`trigger_central.cpp:60`):
```c
getVVTPosition(bankIndex, camIndex):
  // VVT 角度 = 凸轮齿实际角度 - 预期角度
  // 1秒超时: 超过1秒无新数据 → 返回无效
```

## 5.4 调校步骤

### ETB PID 调校

1. **设置安全限制**：先设置最小/最大节气门位置
2. **校准前馈表**：使用 bias 表测量不同目标位置所需的占空比
3. **启用自整定**：让 ECU 自动识别系统动态特性
4. **微调 PID**：根据自整定结果适当调整
5. **验证**：快速踩放踏板，观察跟踪效果

### VVT PID 调校

1. **确认机油压力和温度**：VVT 需要足够机油压力
2. **设置目标角度表**：RPM × 负荷 → 目标凸轮轴角度
3. **从低 Kp 开始**：逐步增加直到有响应
4. **添加 Ki**：消除稳态误差
5. **如果需要添加 Kd**：抑制过冲

## 5.5 常见问题

| 症状 | 可能原因 | 代码相关 |
|------|---------|----------|
| 节气门振荡 | Ki 过大 | `electronic_throttle.cpp:getClosedLoop()` |
| 响应延迟 | Kp 过小 / 前馈不准 | `etbBiasBins` 表需校准 |
| VVT 无响应 | 机油压力不足 / RPM 过低 | `vvt.cpp:onFastCallback()` 检查 `vvtControlMinRpm` |
| VVT 位置漂移 | Ki 过小 | `vvt.cpp` PID 积分项不足 |
