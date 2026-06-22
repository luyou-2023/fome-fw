/**
 * @file    allsensors.cpp
 * [传感器初始化与开关输入]
 * 初始化MAP解码器和各数字开关输入的硬件消抖。
 *
 * 消抖机制(ButtonDebounce):
 *   数字开关(AC按钮/压力开关/油压开关)在物理通断时
 *   会产生短暂的机械抖动(ms级),直接读取会得到错误状态。
 *   ButtonDebounce在检测到电平变化后等待15ms再做最终判定。
 *
 * @brief 传感器初始化顶层函数,在启动流程中调用
 *
 * @date Nov 15, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

static ButtonDebounce acDebounce("ac_switch");
static ButtonDebounce acPressureDebounce("ac_pressure_switch");
static ButtonDebounce oilPressureSwitchDebounce("oil_pressure_switch");

void initSensors() {
	initMapDecoder();
	acDebounce.init(MS2NT(15), engineConfiguration->acSwitch, engineConfiguration->acSwitchMode);
	acPressureDebounce.init(
			MS2NT(15), engineConfiguration->acPressureSwitch, engineConfiguration->acPressureSwitchMode);
	oilPressureSwitchDebounce.init(
			MS2NT(15), engineConfiguration->oilPressureSwitch, engineConfiguration->oilPressureSwitchMode);
}

bool getAcToggle() {
	return acDebounce.readPinState();
}

bool hasAcToggle() {
	return isBrainPinValid(engineConfiguration->acSwitch);
}

bool hasAcPressure() {
	return isBrainPinValid(engineConfiguration->acPressureSwitch);
}

bool getAcPressure() {
	return acPressureDebounce.readPinState();
}

bool hasOilPressureSwitch() {
	return isBrainPinValid(engineConfiguration->oilPressureSwitch);
}

bool getOilSwitchState() {
	return oilPressureSwitchDebounce.readPinState();
}
