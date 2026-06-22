# 传感器子系统文档

## 1. 架构概览

传感器系统采用**流水线架构**，每个阶段处理特定任务：

```
原始数据源 → ADC/GPIO/CAN/SPI/I2C → 转换函数 → 值存储 → 注册表 → 消费者
  硬件层           硬件抽象层          计算转换层    核心框架    算法/控制层
```

### 关键抽象

- **`Sensor`** (`core/sensor.h`): 所有传感器的抽象基类。提供静态 `get(SensorType)` 方法注册表查询。
- **`SensorResult`**: `expected<float>` 类型别名，要么返回有效浮点值，要么返回错误码。
- **`FunctionalSensor`** (`core/functional_sensor.h`): 最常见的传感器基类，组合了一个 `SensorConverter`。
- **`SensorConverter`** (`converters/sensor_converter_func.h`): 转换函数接口，`convert(float raw) → SensorResult`。

### 传感器类型定义 (`sensor_type.h`)

定义了150+种传感器类型枚举，包括：
- 温度类: CLT冷却液温度、IAT进气温度、机油温度、燃油温度
- 压力类: MAP进气压力、机油压力、燃油压力、大气压力
- 位置类: TPS节气门位置、油门踏板位置
- 气体类: Lambda1-4氧传感器
- 速度类: 车速、涡轮转速
- 电气类: 电池电压、传感器5V参考电压

## 2. 数据流详解

### ADC传感器流水线 (最典型)

```
硬件ADC寄存器 (12位或16位整数)
  → AdcProvider::getVoltage()  (MCU电压, 如3.3V)
  → 分压系数缩放 (voltsPerAdcVolt, 如外部电阻分压)
  → Biquad低通滤波 (可配置截止频率)
  → FunctionalSensor::postRawValue()
    → SensorConverter::convert() (转换函数)
      → ResistanceFunc: 电压→电阻 (Steinhart-Hart)
      → ThermistorFunc: 电阻→温度
      → LinearFunc: 线性映射
    → StoredValueSensor::setValidValue()
  → 带时间戳的值存储
```

### 频率传感器流水线

```
GPIO边沿中断 (EXTI)
  → FrequencySensor::onEdge() (中断上下文)
    → 1/Δt 计算瞬时频率
    → Biquad低通滤波
    → postRawValue()
    → 转换函数 (如 FlexConverter: 频率→乙醇含量%)
  → StoredValueSensor
```

### CAN传感器流水线

```
CAN接收帧
  → CanSensorBase::acceptFrame() (ID过滤)
  → CanSensorBase::decodeFrame() (协议解析)
  → setValidValue() / invalidate()
  → StoredValueSensor
```

## 3. 转换函数详解

### 线性转换 (`converters/linear_func.h`)

```c
// y = a * x + b
// 从两个校准点(in1,out1),(in2,out2)计算a和b
// 支持输入分频 (用于外部分压电阻)
// 输出钳位到[minOutput, maxOutput]
// 斜率a为负时高低错误码自动翻转
```

**用途**: MAP传感器(0.5-4.5V → 20-250kPa)、油压传感器、通用电压→工程量转换。

### 电阻转换 (`converters/resistance_func.h`)

```c
// 通过分压电路计算未知电阻值
// R = R_pullup / (VCC/V_raw - 1)
// 短路检测: <0.05V → Low错误
// 开路检测: >98% VCC → High错误
// 支持上拉和下拉两种配置
```

**用途**: 将NTC热敏电阻的ADC电压转换为电阻值，供热敏电阻函数使用。

### 热敏电阻转换 (`converters/thermistor_func.h`)

使用 **Steinhart-Hart 方程**，三个校准点计算系数：

```c
// 已知三点: (R1, T1), (R2, T2), (R3, T3)
// 计算: a, b, c 系数
// lnR = log(R_sensor)
// 1/T_K = a + b*lnR + c*(lnR)^3
// T_C = T_K - 273.15
// 输出钳位到[-50°C, 250°C]
```

**为什么选择Steinhart-Hart而非B参数方程**: Steinhart-Hart提供更高的精度（典型误差<0.1°C），特别是在宽温度范围内。B参数方程只是Steinhart-Hart的近似（忽略c项），在极端温度下误差较大。

### 柔性燃油转换 (`converters/flex_sensor.h`)

```c
// 输入: 50-150Hz 方波信号 (来自柔性燃油传感器)
// 50Hz → 0% 乙醇
// 150Hz → 100% 乙醇
// 频率限制检测: 低于50Hz→Low错误, 高于150Hz→High错误
// Biquad低通滤波约1Hz (平滑传感器读数)
```

### 车速转换 (`converters/vehicle_speed_converter.h`)

```c
// pulsesPerKm = 后轴每公里转数 * 最终传动比 * 传感器齿数
// km/h = 频率(Hz) * 3600 / pulsesPerKm
```

## 4. 冗余/回退机制

### 回退传感器 (`core/fallback_sensor.h`)

当主传感器失效时自动切换到备用传感器：
```
Sensor::get(SensorType::Map)
  → 返回 MAP 传感器值(如果有效)
  → 否则返回 MAP 估算值(基于TPS/RPM查表)
```

### 代理传感器 (`core/proxy_sensor.h`)

将一个传感器映射到另一个传感器类型：
```
Sensor::get(SensorType::DriverThrottleIntent)
  → 如果使用电子节气门，返回踏板位置
  → 如果使用拉线节气门，返回TPS位置
```

### 冗余传感器 (`redundant_sensor.cpp`)

同一物理量有多个传感器时，比较并验证：
- 两个传感器读数偏差在阈值内 → 使用平均值
- 偏差过大 → 判定为传感器故障

## 5. 常用传感器配置

### 冷却液温度 (CLT)
- 典型: NTC热敏电阻 + 上拉电阻
- 常用曲线: Dodge传感器(-40°C@336.6kΩ, 120°C@390Ω) 或 通用NTC(-20°C@18kΩ, 120°C@100Ω)
- ADC采样率: 500Hz (慢速ADC)

### 进气温度 (IAT)
- 与CLT相同的硬件拓扑
- 使用同一套Steinhart-Hart校准曲线
- 响应比CLT快(传感器裸露在进气道中)

### 进气压力 (MAP)
- 典型: 0-5V模拟输出绝对压力传感器
- 线性映射: 0.5V → 20kPa, 4.5V → 250kPa
- 支持Speed-Density和MAF两种空气量计算模式

### 宽域氧传感器 (Lambda)
- 模拟输入: 通过线性函数配置电压→AFR映射
- CAN输入: 支持AEM X-Series和FOME内部宽域控制器
- 用于闭环燃油控制

## 6. 诊断和自我检测

### 超时检测 (`core/stored_value_sensor.h`)

每个传感器值都带有时间戳。如果超过可配置的超时周期没有新数据，`get()` 返回 `UnexpectedCode::Timeout`。这在发动机停止转动时特别有用——传感器不会报告错误的值，而是明确表示数据已过时。

### 范围检查

转换函数在输出时进行范围钳位和错误码转换。例如热敏电阻函数在温度<-50°C或>250°C时返回错误，这通常指示传感器短路或开路。

### ADC故障检测

ADC子系统检测：
- **对地短路**: 电压接近0V (<0.05V)
- **对电源短路**: 电压接近VCC (>98%)
- **通道冲突**: 同一ADC通道被多个传感器订阅
