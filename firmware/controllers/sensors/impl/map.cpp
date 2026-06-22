/**
 * @file map.cpp
 * [MAP传感器解码器]
 * 初始化MAP传感器的解码逻辑。
 *
 * 核心功能: 固定气压补偿
 * 如果设置了useFixedBaroCorrFromMap,在ECU启动时读取一次MAP值
 * 作为大气压力基准(假设发动机未启动,歧管压力=大气压力)
 * 用于高海拔补偿: 海拔高→气压低→减少喷油(空气稀薄)
 *
 * 注意: 这种方法要求在平坦地面、发动机未运行时上电
 * 否则读取的MAP值不能代表大气压力
 *
 * @author Andrey Belomutskiy, (c) 2012-2020
 */
#include "pch.h"

/**
 * This function checks if Baro/MAP sensor value is inside of expected range
 * @return unchanged mapKPa parameter or NaN
 */
static float validateBaroMap(float mapKPa) {
	// Highest interstate is the Eisenhower Tunnel at 11158 feet -> 66 kpa
	// Lowest point is the Dead Sea, -1411 feet -> 106 kpa
	if (std::isnan(mapKPa) || mapKPa > 110 || mapKPa < 60) {
		warning(ObdCode::OBD_Barometric_Press_Circ, "Invalid start-up baro pressure = %.2fkPa", mapKPa);
		return NAN;
	}
	return mapKPa;
}

void initMapDecoder() {
	if (engineConfiguration->useFixedBaroCorrFromMap) {
		// Read initial MAP sensor value and store it for Baro correction.
		float storedInitialBaroPressure = Sensor::get(SensorType::MapSlow).value_or(101.325);
		efiPrintf("Get initial baro MAP pressure = %.2fkPa", storedInitialBaroPressure);
		// validate if it's within a reasonable range (the engine should not be spinning etc.)
		storedInitialBaroPressure = validateBaroMap(storedInitialBaroPressure);
		if (!std::isnan(storedInitialBaroPressure)) {
			// TODO: do literally anything other than this
			Sensor::setMockValue(SensorType::BarometricPressure, storedInitialBaroPressure);
		} else {
			efiPrintf("The baro pressure is invalid. The fixed baro correction will be disabled!");
		}
	}
}
