# 第六章：Miata MX5 快速入门

## 6.1 准备工作

### 所需工具
- BMM Miata ECU + 宽域适配线束 + 选件口尾线
- 博世 LSU 4.9 氧传感器（⚠️ 必须正品，推荐零件号 17025/17212/17123/17217）
- 3米硅胶真空管（5/32" 或 4mm 内径）
- 4mm 直通接头
- 正时灯
- USB 线 + 笔记本电脑（已装 TunerStudio）

## 6.2 拆除原厂 ECU

### 各车型位置
- **左驾 NB**：踏板上方，转向柱旁
- **90-93 左驾 NA / 右驾 NA/NB**：副驾脚坑地毯下
- **94-97 右驾 NA**：乘客座椅后方地毯下

## 6.3 安装宽域氧传感器

1. 找到排气管上的原厂氧传感器（最靠近发动机的那颗）
2. 拧下更换为 LSU 4.9
3. 将线束通过防火墙引入驾驶舱
4. 连接到选件口尾线 → ECU

**代码处理流程—氧传感器信号：**

`firmware/controllers/sensors/impl/AemXSeriesLambda.cpp` — CAN 宽域氧传感器解码：
```c
// 支持 AEM X-Series 和 FOME 内部宽域
// CAN ID 过滤 → 数据帧解码 → lambda 值提取
// decodeAemXSeries(): data16[0] * 0.0001
// 检查 valid bit(7) 和 fault bit(6)
```

或者使用模拟输入（0-5V 线性映射），见 `firmware/controllers/sensors/impl/ego.cpp`：
```c
// 预置常见宽域控制器映射:
// BPSX D1:       0-5V → 9-19 AFR
// Innovate MTX-L: 0-5V → 7.35-22.39 AFR
// 14Point7 Free: 0-5V → 9.996-19.992 AFR
```

## 6.4 连接 MAP 管路

1. 在节气门后的进气歧管上找真空接口
2. 连接真空管，穿过防火墙到 ECU
3. 连接到 ECU 壳体内的 MAP 传感器
4. 可选：在真空管上加管夹

**代码处理流程—MAP 传感器：**

MAP 信号经过 ADC → 线性函数转换 → 用于 Speed-Density 计算。

`firmware/controllers/algo/airmass/speed_density_airmass.cpp`：
```c
// 速度密度法进气量计算:
// airMass = MAP × VE × displacement / (R × IAT_K)
// MAP = 进气歧管绝对压力
// VE = 充气效率 (3D表: RPM × 负荷)
// IAT = 进气温度
```

## 6.5 NA6 1.6L 特殊步骤

### TPS 更换
NA6 1.6L 手动挡使用开关式 TPS（非可变），需更换为可变 TPS：
- 断开原厂 TPS 连接器（⚠️ 防止短路）
- BMM ECU 附带 KIA TPS，直接对插
- 如使用其他 TPS，接线如下：

| 功能 | 线色 |
|------|------|
| 信号 | 绿/白 |
| 地线 | 黑/绿 |
| 5V 参考 | 红 |

### IAT 传感器
NA6 使用 AFM（空气流量计），内含 IAT。拆除 AFM 后需加装独立 IAT：
- 推荐 GM IAT 传感器（FOME 已有配置）
- IAT 是电阻型传感器，两根线不分正负

**代码位置：** `firmware/init/sensor/init_thermistors.cpp` — IAT 热敏电阻初始化

### 燃油泵跳线
拆除 AFM 后，需在 AFM 连接器上加跳线让 ECU 控制燃油泵。

**代码位置：** `firmware/controllers/modules/fuel_pump/fuel_pump.cpp` — 燃油泵控制逻辑
```c
// 燃油泵在以下条件下启动:
// 1. 点火开关 ON (预注油几秒)
// 2. RPM > 0 (发动机旋转)
// 3. 未触发燃油切断(如碰撞熄火)
```

## 6.6 TunerStudio 设置

1. 创建新项目 → Detect → 选择 COM 口
2. 选择显示单位：推荐 Lambda
   - Lambda = 1.0 = 理论空燃比（汽油 14.7:1）
   - Lambda = 1.1 = 稀 10%，Lambda = 0.9 = 浓 10%
3. 设置点火模式为 **固定 10 度**（用于设正时）

## 6.7 设基础正时

1. 连接正时灯到 1 缸火花塞线
2. 发动机怠速运行
3. 正时灯照射曲轴皮带轮
4. 调整 Trigger Advance Angle 直到正时标记对齐
5. 将点火模式从 "fixed" 改回 "dynamic"

**代码处理流程—点火提前角：**

`firmware/controllers/engine_cycle/spark_logic.cpp:46` — `getSparkAngle()`：
```c
// 最终点火提前角 = 基础提前角 + 修正值
// 基础提前角来自 ignitionTable (3D: RPM × 负荷)
// 修正包括: IAT/CLT修正 + 爆震推迟 + 怠速调整 + ALS
// 结果钳位到 [minimumIgnitionTiming, maximumIgnitionTiming]
```

`trigger_central.cpp:36` — 全局触发角度偏移 `globalTriggerAngleOffset`：
```c
// toEngPhase(): 将触发域角度转换为发动机域角度
// engineAngle = triggerAngle - tdcPosition - globalTriggerAngleOffset + phaseAdjustment
// 这个偏移量就是在 TunerStudio 中调整的 "Trigger Advance Angle"
```

## 6.8 VE 表调校

### 什么是 VE 表

VE（Volumetric Efficiency，容积效率）表是一个 3D 表：
- 横轴：RPM
- 纵轴：发动机负荷（MAP）
- 值：充气效率百分比

ECU 用 VE 值计算喷油量：
```
fuelMass = airMass / targetAFR
airMass = MAP × VE × displacement / (R × IAT)
```

### 调校方法

**方法一：手动调校**
1. 看 Lambda 实际值 vs Target Lambda 表
2. 如果 Lambda 1.1 但目标是 1.0，对应 VE 格增加 10%

**方法二：AutoTune（推荐，需要 TS 付费版）**
1. 关闭 Closed Loop Fuel Correction 和 DFCO
2. 启动 Tune Analyze Live!
3. 平稳驾驶各种工况
4. 逐步降低 Cell Change Resistance 精调

**方法三：马力机调校**
最精确但成本最高

**代码处理流程—燃油计算：**

`firmware/controllers/algo/fuel_math.cpp:164` — `getBaseFuelMass()`：
```c
// 1. 选择空气量模型: Speed-Density / MAF / Alpha-N
// 2. getAirmass(rpm) → 每缸空气量
// 3. getCycleFuel(airMass) → airMass / targetAFR → 燃油质量
// 4. × globalFuelCorrection
```

`firmware/controllers/algo/fuel_math.cpp:294` — `getCycleInjectionMass()`：
```c
// 最终燃油量 = 基础燃油 + 起动修正 + 运行修正(IAT/CLT/Baro)
//            + 加速补偿(TPS变化率) - DFCO(减速断油)
// → injectorModel 燃油质量→喷油脉宽(含无效时间修正)
```

`firmware/controllers/algo/fuel/injector_model.cpp` — 喷油器模型：
```c
// 喷油脉宽 = 燃油质量 / 喷油器流量 + 无效时间(dead time)
// 无效时间随电池电压变化: 电压高→电磁力强→开启快→无效时间短
```
