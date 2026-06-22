/**
 * @file functional_sensor.cpp
 * [功能性传感器]
 * ADC/频率传感器数据进入传感器框架的核心入口。
 * 外部驱动(ADC订阅/频率传感器中断等)调用postRawValue(),
 * 传入原始读数,通过转换函数处理后存入带时间戳的值存储。
 *
 * 数据流:
 *   硬件驱动 → postRawValue(raw_value, timestamp)
 *     → m_function->convert(raw) (电压→电阻→温度等)
 *     → setValidValue(result, timestamp) 或 invalidate(error)
 *
 * 注意: 值和有效位的设置顺序需要防止数据竞争
 *   先存值再设置有效位,确保读取时不会读到旧值+新有效位
 */

#include "functional_sensor.h"

void FunctionalSensor::postRawValue(float inputValue, efitick_t timestamp) {
	// If no function is set, this sensor isn't valid.
	if (!m_function) {
		invalidate(UnexpectedCode::Configuration);
		return;
	}

	m_rawValue = inputValue;

	auto r = m_function->convert(inputValue);

	// This has to happen so that we set the valid bit after
	// the value is stored, to prevent the data race of reading
	// an old invalid value
	if (r.Valid) {
		setValidValue(r.Value, timestamp);
	} else {
		invalidate(r.Code);
	}
}
