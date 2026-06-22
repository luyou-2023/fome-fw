/**
 * @author Andrey Belomutskiy, (c) 2012-2020
 *
 * EGO Exhaust Gas Oxygen, also known as AFR Air/Fuel Ratio :)
 * [氧传感器配置]
 * 预置常见宽域氧控制器的模拟信号电压→AFR映射。
 * 所有映射都是线性的:v1电压→value1 AFR, v2电压→value2 AFR。
 *
 * 支持型号:
 *   BPSX D1:         0-5V → 9-19 AFR
 *   Innovate MTX-L:  0-5V → 7.35-22.39 AFR
 *   14Point7 Free:   0-5V → 9.996-19.992 AFR
 *   PLX:             0-5V → 10-20 AFR
 *
 */
#include "pch.h"

void setEgoSensor(ego_sensor_e type) {
	auto sensor = &engineConfiguration->afr;

	switch (type) {
		case ES_BPSX_D1:
			sensor->v1 = 0;
			sensor->value1 = 9;
			sensor->v2 = 5;
			sensor->value2 = 19;
			break;

		case ES_Innovate_MTX_L:
			sensor->v1 = 0;
			sensor->value1 = 7.35;
			sensor->v2 = 5;
			sensor->value2 = 22.39;
			break;
		case ES_14Point7_Free:
			sensor->v1 = 0;
			sensor->value1 = 9.996;
			sensor->v2 = 5;
			sensor->value2 = 19.992;
			break;
		case ES_PLX:
			sensor->v1 = 0;
			sensor->value1 = 10;
			sensor->v2 = 5;
			sensor->value2 = 20;
			break;
		default:
			firmwareError(ObdCode::CUSTOM_EGO_TYPE, "Unexpected EGO %d", type);
			break;
	}
}
