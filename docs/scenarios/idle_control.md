# 怠速控制场景分析

## 1. 场景描述

发动机在松开油门、变速箱空挡或离合器踩下时维持目标怠速转速。核心挑战在于各种负载变化（空调、发电机、动力转向）对RPM的干扰。

## 2. 控制架构

双重控制回路:

```
怠速目标RPM
  ↓
┌─ IdleTargetController (确定阶段和目标) ─┐
│  查表: 水温→目标RPM                      │
│  加载: AC提升, Lua修正                    │
└──────────────────────────────────────────┘
  ↓
┌─ IdleController (执行控制) ──────────────┐
│  开环: 基本IAC位置 + 修正项              │
│  闭环: RPM PID (仅在Idling阶段)          │
│  定时: 点火提前角辅助PID                 │
└──────────────────────────────────────────┘
  ↓
IAC执行器 (怠速空气控制阀 / 电子节气门微开)
  + 点火定时微调
```

## 3. 状态转换

```
                    Cranking
                        ↓
              CrankToIdleTaper ← afterCrankingIACtaperDuration
                        ↓
Running ←────────→ Idling ←─────────→ Coasting
                              ← throttle off
                                      RPM > exit threshold
```

## 4. 关键算法

### 开环位置

```c
getRunningOpenLoop(rpm, clt, tps):
  // 基本位置 (cltIdleCorr表)
  // + AC补偿 (acIdleExtraOffset)
  // + 风扇补偿 (fan1ExtraIdle, fan2ExtraIdle)
  // + Lua补偿
  // + ALS补偿
  // + TPS渐变 (突然松油门时缓慢减少)
  // + RPM渐变 (目标RPM变化时缓慢跟随)
```

**为什么需要这么多叠加项**: 怠速是一个多变量干扰系统。空调压缩机接合需要额外功率，发电机充电需要机械能，冷却风扇开启也增加负载。每种负载都需要不同量的额外进气补偿。

### 空燃比修正（隐含）

在怠速PID调节IAC（进气量）的同时，燃油系统也会根据进气量的变化自动调整喷油脉宽，维持理论空燃比（14.7:1）。IAC和燃油的协调是自动的——燃油系统跟踪进气传感器（MAF或MAP）的变化。

### 点火定时辅助

```c
getIdleTimingAdjustment(rpm, rpmRate):
  // 独立PID, P参数通常较小
  // 提前角↑ → 燃烧更早 → 扭矩↑ → RPM↑
  // 提前角↓ → 燃烧推迟 → 扭矩↓ → RPM↓
  // 响应时间: < 1个发动机周期 (远快于IAC)
```

**IAC vs 点火定时**: IAC（进气）响应慢但调节范围大；点火定时响应快但调节范围小（通常±5°提前角）。两者配合形成"快慢结合"的策略——点火定时处理瞬时干扰，IAC处理稳态偏差。

### PID积分复位策略

```c
if (phase != Idling) {
    // 离开Idling状态时复位PID
    // 如果积分项为负(进气不足)，也复位
    // 避免下次进入Idling时积分器残留误差
}
```

**为什么复位负积分**: 如果因为高负载导致积分器累积了大量正值（需要更多进气），回到Idling时可以快速恢复。但如果积分器是负值（进气过多），回到Idling时会导致怠速过低甚至熄火，所以负积分需要复位。

## 5. 常见问题

| 症状 | 可能原因 | 排查方向 |
|------|---------|---------|
| 怠速不稳 | PID参数不当 | 检查Ki/Kp;检查IAC阀工作 |
| 怠速过高 | 基本位置设置过高 | 检查cltIdleCorr/manIdlePosition |
| 松油门熄火 | Coasting策略不当 | 检查iacCoasting表/AirTaper设置 |
| 开空调熄火 | AC补偿不足 | 增加acIdleRpmBump/acIdleExtraOffset |
