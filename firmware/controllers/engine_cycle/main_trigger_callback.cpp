/**
 * @file    main_trigger_callback.cpp
 * @brief   Main logic is here!
 *
 * See http://rusefi.com/docs/html/
 *
 * @date Feb 7, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "pch.h"

#if EFI_PRINTF_FUEL_DETAILS
bool printFuelDebug = false;
#endif // EFI_PRINTF_FUEL_DETAILS

#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT

#include "spark_logic.h"

/* ===== 燃油事件处理 =====
 * 每个触发齿事件都会调用此函数
 * 如果燃油调度还未准备好（首次同步时），立即重建调度
 * 这是为了在发动机刚刚完成同步时能立即喷油，而不是等到下一次快速回调
 */
static void handleFuel(const EnginePhaseInfo& phase) {
	ScopePerf perf(PE::HandleFuel);

	/* 限速管理器禁止喷油时直接返回(如减速断油DFCO) */
	if (!getLimpManager()->allowInjection().value) {
		return;
	}

	/* 快速回调中也会调用addFuelEvents()，但首次同步时可能还未执行
	 * 这里做双重保证：只要调度未就绪就立即初始化
	 * 避免刚起动时第一个喷油事件延迟
	 */
	FuelSchedule* fs = getFuelSchedule();
	if (!fs->isReady) {
		fs->addFuelEvents();
	}

	/* onTriggerTooth: 根据当前触发齿相位检查并激活已调度的喷油事件
	 * 每个齿事件都可能触发喷油嘴开始或停止喷油
	 */
	fs->onTriggerTooth(phase);
}

/**
 * This is the main trigger event handler.
 * Both injection and ignition are controlled from this method.
 */
/* ===== 主触发回调 =====
 * 每次触发解码器成功解码一个触发齿事件后调用
 * trgEventIndex: 当前周期内的触发齿索引(0=同步点)
 * phase: 当前和下一个发动机相位信息(角度/时间)
 * 这是ECU所有燃油和点火调度决策的入口点
 * 在中断上下文中被调用，需要快速完成
 */
void mainTriggerCallback(uint32_t trgEventIndex, const EnginePhaseInfo& phase) {
	ScopePerf perf(PE::MainTriggerCallback);

	/* 固件严重错误(如Flash校验失败) → 停止所有发动机控制
	 * 宁可停机也不冒险损坏发动机
	 */
	if (hasFirmwareError()) {
		/**
		 * In case on a major error we should not process any more events.
		 */
		return;
	}

	/* RPM为0时说明发动机还未开始旋转(起动机未拖动)
	 * 此时不处理任何燃油/点火事件
	 */
	float rpm = engine->rpmCalculator.getCachedRpm();
	if (rpm == 0) {
		// this happens while we just start cranking

		// todo: check for 'trigger->is_synchnonized?'
		return;
	}

	/* 同步点(trgEventIndex=0)是新发动机周期的开始
	 * 此时检查触发配置是否在运行中被修改(如TunerStudio在线调整)
	 * 如果配置已变化，需要重建所有调度以确保一致性
	 */
	if (trgEventIndex == 0) {
		if (getTriggerCentral()->checkIfTriggerConfigChanged()) {
			getIgnitionEvents()->isReady = false; // we need to rebuild complete ignition schedule
			getFuelSchedule()->invalidate();
			// moved 'triggerIndexByAngle' into trigger initialization (why was it invoked from here if it's only about
			// trigger shape & optimization?) see updateTriggerWaveform() -> prepareOutputSignals()

			// we need this to apply new 'triggerIndexByAngle' values
			engine->periodicFastCallback();
		}
	}

	/* 逐个通知所有发动机模块执行相位事件
	 * 使用模板apply_all避免虚函数开销(-fno-rtti)
	 * 每个模块可以在这里执行需要实时性的操作
	 * 如: 爆震采样窗口开关、VVT位置更新等
	 */
	engine->engineModules.apply_all([=](auto& m) { m.onEnginePhase(rpm, phase); });

	/**
	 * For fuel we schedule start of injection based on trigger angle, and then inject for
	 * specified duration of time
	 */
	handleFuel(phase);

	/**
	 * For spark we schedule both start of coil charge and actual spark based on trigger angle
	 */
	onTriggerEventSparkLogic(phase);
}

#endif /* EFI_ENGINE_CONTROL */
