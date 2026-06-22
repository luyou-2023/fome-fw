/*
 * [lua_getchannel.cpp - Lua获取通道值]
 * 实现Lua脚本通过名称获取ECU传感器/计算通道值的功能。
 * 使用djb2哈希匹配通道名称，支持燃油流量、AFR、喷油脉宽等。
 * 关键参数: 通道名称字符串, 返回expected<float>值
 */
#include "pch.h"

#include "lua_getchannel.h"
#include "fuel_math.h"

expected<float> getChannelByName(const char* name) {
	switch (djb2lowerCase(name)) {
		case djb2lowerCase("FuelFlow"):
			return engine->module<TripOdometer>()->getConsumptionGramPerSecond();
		case djb2lowerCase("AFR"):
			return (Sensor::getOrZero(SensorType::Lambda1) * engineConfiguration->stoichRatioPrimary);
		case djb2lowerCase("InjectorDutyCycle"):
			return getInjectorDutyCycle(Sensor::getOrZero(SensorType::Rpm));
		case djb2lowerCase("InjectorPulseWidth"):
			return static_cast<float>(engine->outputChannels.actualLastInjection);
		default:
			return unexpected;
	}
}
