/**
 * @author Matthew Kennedy, (c) 2019
 * [热敏电阻转换函数]
 * 使用Steinhart-Hart方程将NTC热敏电阻的电阻值转换为温度。
 *
 * Steinhart-Hart方程: 1/T = a + b*ln(R) + c*(ln(R))³
 * 需要三个校准点(T₁,R₁)(T₂,R₂)(T₃,R₃)求解三个系数a,b,c
 * 比简单的B参数方程精度更高(覆盖更宽温度范围)
 *
 * 温度范围: -50°C ~ +250°C
 * 低于-50°C判定为故障(Low),可能是传感器短路
 * 高于250°C判定为故障(High),可能是传感器开路或过热
 */

#include "pch.h"

#include "thermistor_func.h"

#include <math.h>

SensorResult ThermistorFunc::convert(float ohms) const {
	// This resistance should have already been validated - only
	// thing we can check is that it's non-negative
	if (ohms <= 0) {
		return UnexpectedCode::Low;
	}

	float lnR = logf(ohms);

	float lnR3 = lnR * lnR * lnR;

	float recip = m_a + m_b * lnR + m_c * lnR3;

	float kelvin = 1 / recip;

	float celsius = convertKelvinToCelcius(kelvin);

	// bounds check result - please don't try to run rusEfi when colder than -50C
	// high end limit is required as this could be an oil temp sensor on an
	// air cooled engine
	if (celsius < -50) {
		return UnexpectedCode::Low;
	}

	if (celsius > 250) {
		return UnexpectedCode::High;
	}

	return celsius;
}

/* Steinhart-Hart系数计算:
 * 从三个已知的(T,R)校准点求解a,b,c
 * 这是线性最小二乘问题,解析求解:
 *   设 yᵢ = 1/Tᵢ, lᵢ = ln(Rᵢ)
 *   则 c = ((u₃-u₂)/(l₃-l₂)) / (l₁+l₂+l₃)
 *   其中 u₂ = (y₂-y₁)/(l₂-l₁), u₃ = (y₃-y₁)/(l₃-l₁)
 *   然后 b = u₂ - c*(l₁²+l₁*l₂+l₂²)
 *   最后 a = y₁ - (b + c*l₁²)*l₁
 */
void ThermistorFunc::configure(thermistor_conf_s& cfg) {
	// https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
	float l1 = logf(cfg.resistance_1);
	float l2 = logf(cfg.resistance_2);
	float l3 = logf(cfg.resistance_3);

	float y1 = 1 / convertCelsiusToKelvin(cfg.tempC_1);
	float y2 = 1 / convertCelsiusToKelvin(cfg.tempC_2);
	float y3 = 1 / convertCelsiusToKelvin(cfg.tempC_3);

	float u2 = (y2 - y1) / (l2 - l1);
	float u3 = (y3 - y1) / (l3 - l1);

	m_c = ((u3 - u2) / (l3 - l2)) / (l1 + l2 + l3);
	m_b = u2 - m_c * (l1 * l1 + l1 * l2 + l2 * l2);
	m_a = y1 - (m_b + l1 * l1 * m_c) * l1;
}
