/**
 * @file thermistors.cpp
 * [热敏电阻配置]
 * 预置常见NTC热敏电阻的Steinhart-Hart校准曲线。
 * 在发动机配置中调用,将特定型号的传感器曲线设置到配置中。
 *
 * Steinhart-Hart方程:
 *   1/T = a + b*ln(R) + c*(ln(R))^3
 * 已知三个温度点的电阻值即可计算a,b,c三个系数。
 *
 * 这里不直接计算方程,而是提供三对(T,R)校准点,
 * 由ThermistorFunc在运行时计算系数。
 *
 * @date Feb 17, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

void setDodgeSensor(ThermistorConf* thermistorConf, float pullup) {
	thermistorConf->config = {-40, 30, 120, 336660, 7550, 390, pullup};
}

// todo: better method name?
void setCommonNTCSensor(ThermistorConf* thermistorConf, float pullup) {
	/**
	 * 18K Ohm @ -20C
	 * 2.1K Ohm @ 24C
	 * 294 Ohm @ 80C
	 * http://www.rexbo.eu/hella/coolant-temperature-sensor-6pt009107121?c=100334&at=3130
	 */
	thermistorConf->config = {-20, 23.8889, 120, 18000, 2100, 100, pullup};
}
