/**
 * @author Andrey Belomutskiy, (c) 2012-2020
 * [TPS校准]
 * 通过TunerStudio指令执行TPS和踏板位置传感器的自校准。
 * grabTPSIsClosed/grabTPSIsWideOpen: 记录全关/全开位置的ADC值
 * grabPedalIsUp/grabPedalIsWideOpen: 发布校准值到TunerStudio输出通道
 *
 * 校准后的值存储在engineConfiguration中,持久化到Flash。
 */
#include "pch.h"

void grabTPSIsClosed() {
#if EFI_PROD_CODE
	engineConfiguration->tpsMin = convertVoltageTo10bitADC(Sensor::getRaw(SensorType::Tps1));
#endif /* EFI_PROD_CODE */
}

void grabTPSIsWideOpen() {
#if EFI_PROD_CODE
	engineConfiguration->tpsMax = convertVoltageTo10bitADC(Sensor::getRaw(SensorType::Tps1));
#endif /* EFI_PROD_CODE */
}

void grabPedalIsUp() {
	/**
	 * search for 'maintainConstantValue' to find how this TS magic works
	 */
	engine->outputChannels.calibrationMode = (uint8_t)TsCalMode::PedalMin;
	engine->outputChannels.calibrationValue = Sensor::getRaw(SensorType::AcceleratorPedalPrimary);
	engine->outputChannels.calibrationValue2 = Sensor::getRaw(SensorType::AcceleratorPedalSecondary);
}

void grabPedalIsWideOpen() {
	engine->outputChannels.calibrationMode = (uint8_t)TsCalMode::PedalMax;
	engine->outputChannels.calibrationValue = Sensor::getRaw(SensorType::AcceleratorPedalPrimary);
	engine->outputChannels.calibrationValue2 = Sensor::getRaw(SensorType::AcceleratorPedalSecondary);
}
