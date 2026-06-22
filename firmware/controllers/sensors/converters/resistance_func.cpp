/**
 * @author Matthew Kennedy, (c) 2019
 * [电阻转换函数]
 * 通过分压电路(上拉/下拉电阻)计算传感器的电阻值。
 * 用于: NTC热敏电阻(温度传感器)、油门位置电位器等。
 *
 * 分压公式: R_sensor = R_pullup / (VCC / V_out - 1)
 * (传感器一端接VCC,另一端通过上拉电阻接地,测量中间点电压)
 *
 * 故障检测: 检测传感器短路(电压<0.05V)和开路(电压>98%VCC)
 * 下拉配置: 如果传感器在上拉侧,取反电压后计算
 */

#include "resistance_func.h"

void ResistanceFunc::configure(float supplyVoltage, float pullupResistor, bool isPulldown) {
	m_pullupResistor = pullupResistor;
	m_supplyVoltage = supplyVoltage;
	m_isPulldown = isPulldown;
}

SensorResult ResistanceFunc::convert(float raw) const {
	// If the voltage is very low, the sensor is a dead short.
	if (raw < 0.05f) {
		return UnexpectedCode::Low;
	}

	// If the voltage is very high (98% VCC), the sensor is open circuit.
	if (raw > (m_supplyVoltage * 0.98f)) {
		return UnexpectedCode::High;
	}

	if (m_isPulldown) {
		// If the sensor is on the high side (fixed resistor is pulldown),
		// invert the voltage so the math comes out correctly
		raw = m_supplyVoltage - raw;
	}

	// Voltage is in a sensible range - convert
	float resistance = m_pullupResistor / (m_supplyVoltage / raw - 1);

	return resistance;
}
