/**
 * @file	engine.cpp
 *
 * [发动机状态管理]
 * Engine类是ECU固件的核心"上帝对象",聚合所有子系统和状态
 * 虽然违反了单一职责原则,但在嵌入式C++的约束下(-fno-rtti,无异常)
 * 这是一种务实的实现方式——所有模块通过Engine类共享数据和状态
 *
 * 核心功能:
 * 1. periodicFastCallback(250Hz): 燃油/点火计算和模块更新
 * 2. periodicSlowCallback(20Hz):  传感器/看门狗/配置检查
 * 3. 触发同步管理: 同步建立/丢失/恢复
 * 4. 安全看门狗: 发动机停止时自动关闭喷油点火
 *
 * This might be a http://en.wikipedia.org/wiki/God_object but that's best way I can
 * express myself in C/C++. I am open for suggestions :)
 *
 * @date May 21, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#include "trigger_central.h"
#include "fuel_math.h"
#include "speed_density.h"

#include "perf_trace.h"
#include "backup_ram.h"
#include "idle_thread.h"
#include "idle_hardware.h"
#include "gppwm.h"
#include "speedometer.h"
#include "boost_control.h"
#include "ac_control.h"
#include "vr_pwm.h"
#if EFI_MC33816
#include "mc33816.h"
#endif // EFI_MC33816

#if EFI_PROD_CODE
#include "trigger_emulator_algo.h"
#include "bench_test.h"
#else
#define isRunningBenchTest() true
#endif /* EFI_PROD_CODE */

#if (BOARD_TLE8888_COUNT > 0)
#include "gpio/tle8888.h"
#endif

#if EFI_ENGINE_SNIFFER
#include "engine_sniffer.h"
extern WaveChart waveChart;
#endif /* EFI_ENGINE_SNIFFER */

void Engine::resetEngineSnifferIfInTestMode() {
#if EFI_ENGINE_SNIFFER
	if (isFunctionalTestMode) {
		// TODO: what is the exact reasoning for the exact engine sniffer pause time I wonder
		waveChart.pauseEngineSnifferUntilNt = getTimeNowNt() + MS2NT(300);
		waveChart.reset();
	}
#endif /* EFI_ENGINE_SNIFFER */
}

void Engine::updateTriggerWaveform() {
#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT
	// we have a confusing threading model so some synchronization would not hurt
	chibios_rt::CriticalSectionLocker csl;

	engine->triggerCentral.updateWaveform();

	if (!engine->triggerCentral.triggerShape.shapeDefinitionError) {
		prepareOutputSignals();
	}
#endif /* EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT */
}

/* ===== 慢速周期回调(20Hz) =====
 * 在主循环中每50ms执行一次,处理不需要高频率的任务
 * 这些任务不适合在中断上下文中执行(耗时较长或需要I2C/SPI通信)
 *
 * 执行内容:
 * 1. 触发配置重新读取(支持热插拔配置变更)
 * 2. 发动机停止检测(触发信号超时)
 * 3. 安全看门狗(无信号时关闭燃油/点火)
 * 4. 慢速传感器读取(开关输入/气压/温度)
 * 5. TPS加速补偿更新
 * 6. 各模块慢速回调(如怠速控制/交流发电机等)
 */
void Engine::periodicSlowCallback() {
#if EFI_SHAFT_POSITION_INPUT
	// Re-read config in case it's changed
	triggerCentral.primaryTriggerConfiguration.update();
	for (int camIndex = 0; camIndex < CAMS_PER_BANK; camIndex++) {
		triggerCentral.vvtTriggerConfiguration[camIndex].update();
	}

	/* 发动机停止检测:
	 * 如果超过阈值时间没有触发信号,认为发动机已停止
	 * engineMovedRecently()检查从最后一次触发事件到现在的时间
	 * 如果超过RPM_LOW_THRESHOLD对应的周期,返回false
	 * 触发同步丢失流程: 重置RPM/使调度无效/模块通知
	 */
	if (!triggerCentral.engineMovedRecently(getTimeNowNt()) && !rpmCalculator.isStopped()) {
		OnTriggerSynchronizationLost();
	}
#endif // EFI_SHAFT_POSITION_INPUT

	/* 安全看门狗: 如果发动机停止,
	 * 关闭喷油器和点火线圈引脚,防止意外喷油/点火
	 * 同时通知各模块执行安全操作
	 */
	efiWatchdog();
	updateSlowSensors();
	checkShutdown();

	/* TPS加速补偿: 基于最新的TPS值更新
	 * TPS变化率 → 加速加浓量计算
	 * 在慢速回调中更新,但实际补偿值在燃油计算时使用
	 */
	module<TpsAccelEnrichment>()->onNewValue(Sensor::getOrZero(SensorType::Tps1));

	/* VR PWM(可变磁阻传感器):
	 * 一些老式曲轴/凸轮轴传感器输出需要自适应阈值处理
	 * VR PWM模块调节信号整形电路的门限电平
	 */
	updateVrPwm();

	/* 氧传感器加热器:
	 * 发动机运行时打开加热器(氧传感器需要工作温度~600°C)
	 * forceO2Heating可强制加热(诊断模式)
	 */
	enginePins.o2heater.setValue(engineConfiguration->forceO2Heating || engine->rpmCalculator.isRunning());
	/* 起动继电器: RPM低于起动阈值时关闭(起动机脱开) */
	enginePins.starterRelayDisable.setValue(Sensor::getOrZero(SensorType::Rpm) < engineConfiguration->cranking.rpm);

	/* GPPWM(通用PWM): 更新所有通用PWM通道
	 * 用于: 散热风扇/燃油泵/水泵等辅助设备
	 * 映射: 传感器值→PWM占空比
	 */
	updateGppwm();

	/* 所有发动机模块的慢速回调
	 * 每个模块在此更新自己的状态:
	 * - 怠速控制器: 检查是否需要切换怠速阶段
	 * - 交流发电机: 更新目标充电电压
	 * - 空调控制器: 检查空调请求和压力开关
	 * - 增压控制器: 更新目标增压值
	 */
	engine->engineModules.apply_all([](auto& m) { m.onSlowCallback(); });

#if (BOARD_TLE8888_COUNT > 0)
	tle8888startup();
#endif

#if EFI_PROD_CODE
	/* LPS25气压传感器更新(通过I2C):
	 * 读取板载气压传感器用于海拔补偿
	 * I2C通信较慢(ms级),放在慢速回调中
	 */
	void baroLps25Update();
	baroLps25Update();
#endif // EFI_PROD_CODE

	/* 更新分段喷射分配比例 */
	engineState.updateSplitInjection();
}

/**
 * We are executing these heavy (logarithm) methods from outside the trigger callbacks for performance reasons.
 * See also periodicFastCallback
 */
void Engine::updateSlowSensors() {
	updateSwitchInputs();

#if EFI_SHAFT_POSITION_INPUT
	float rpm = Sensor::getOrZero(SensorType::Rpm);
	triggerCentral.isEngineSnifferEnabled = rpm < engineConfiguration->engineSnifferRpmThreshold;
#endif // EFI_SHAFT_POSITION_INPUT
}

static bool getClutchUpState() {
	if (isBrainPinValid(engineConfiguration->clutchUpPin)) {
		return engineConfiguration->clutchUpPinInverted ^ efiReadPin(engineConfiguration->clutchUpPin);
	}
	return engine->engineState.lua.clutchUpState;
}

static bool getBrakePedalState() {
	if (isBrainPinValid(engineConfiguration->brakePedalPin)) {
		return efiReadPin(engineConfiguration->brakePedalPin);
	}
	return engine->engineState.lua.brakePedalState;
}

void Engine::updateSwitchInputs() {
	// this value is not used yet
	if (isBrainPinValid(engineConfiguration->clutchDownPin)) {
		engine->engineState.clutchDownState =
				engineConfiguration->clutchDownPinInverted ^ efiReadPin(engineConfiguration->clutchDownPin);
	}
	{
		bool currentState;
		if (hasAcToggle()) {
			currentState = getAcToggle();
		} else {
			currentState = engine->engineState.lua.acRequestState;
		}
		AcController& acController = engine->module<AcController>().unmock();
		if (acController.acButtonState != currentState) {
			acController.acButtonState = currentState;
			acController.timeSinceStateChange.reset();
		}
		if (hasAcPressure()) {
			acController.acPressureSwitchState = getAcPressure();
		}
	}

	engine->engineState.clutchUpState = getClutchUpState();
	engine->engineState.brakePedalState = getBrakePedalState();
}

Engine::Engine() {
	reset();
}

int Engine::getGlobalConfigurationVersion() const {
	return globalConfigurationVersion;
}

void Engine::reset() {
	/**
	 * it's important for wrapAngle() that engineCycle field never has zero
	 */
	engineState.engineCycle = getEngineCycle(FOUR_STROKE_CRANK_SENSOR);
	resetLua();
}

void Engine::resetLua() {
	// todo: https://github.com/rusefi/rusefi/issues/4308
	engineState.lua = {};
	engineState.lua.fuelAdd = 0;
	engineState.lua.fuelMult = 1;
	engineState.lua.luaDisableEtb = false;
	engineState.lua.luaIgnCut = false;
	module<BoostController>().unmock().resetLua();
	ignitionState.luaTimingAdd = 0;
	ignitionState.luaTimingMult = 1;
#if EFI_IDLE_CONTROL
	module<IdleController>().unmock().luaAdd = 0;
	module<IdleTargetController>().unmock().luaAddRpm = 0;
#endif // EFI_IDLE_CONTROL
}

/* ===== 触发同步状态管理 =====
 * OnTriggerStateProperState: 触发成功同步时调用
 *   通知RPM计算器"发动机正在旋转"
 *   用于从停止状态转换到运行状态
 *
 * OnTriggerSynchronizationLost: 同步丢失时调用
 *   通常在发动机关闭或触发信号中断时触发
 *   重置所有与曲轴位置相关的状态,使调度无效
 *   防止在下次起动时基于错误的相位信息喷油/点火
 */

#if EFI_SHAFT_POSITION_INPUT
void Engine::OnTriggerStateProperState(efitick_t nowNt) {
	rpmCalculator.setSpinningUp(nowNt);
}

void Engine::OnTriggerSynchronizationLost() {
	efiPrintf("engine stopped");

	rpmCalculator.setStopSpinning();

	/* 重置所有触发状态:
	 * 触发解码器状态 → 回到未同步状态
	 * 瞬时RPM → 归零
	 * VVT状态 → 所有通道全部重置
	 */
	triggerCentral.triggerState.resetState();
	triggerCentral.instantRpm.resetInstantRpm();

	for (size_t i = 0; i < efi::size(triggerCentral.vvtState); i++) {
		for (size_t j = 0; j < efi::size(triggerCentral.vvtState[0]); j++) {
			triggerCentral.vvtState[i][j].resetState();
		}
	}

	/* 使燃油/点火调度无效:
	 * 下次起动时要重建调度,避免使用旧的dwell角度
	 * 否则可能出现dwell时间错误导致线圈过热或点火能量不足
	 */
	injectionEvents.invalidate();
	engine->ignitionEvents.isReady = false;

	/* 通知所有发动机模块:
	 * 让各模块执行停止逻辑(如复位PID积分器、关闭执行器等)
	 */
	engineModules.apply_all([](auto& m) { m.onEngineStop(); });
}

void Engine::OnTriggerSyncronization(bool wasSynchronized, bool isDecodingError) {
	// TODO: this logic probably shouldn't be part of engine.cpp

	// We only care about trigger shape once we have synchronized trigger. Anything could happen
	// during first revolution and it's fine
	if (wasSynchronized) {
		// 'triggerStateListener is not null' means we are running a real engine and now just preparing trigger shape
		// that's a bit of a hack, a sweet OOP solution would be a real callback or at least 'needDecodingErrorLogic'
		// method?
		if (isDecodingError) {
#if EFI_PROD_CODE
			if (engineConfiguration->verboseTriggerSynchDetails ||
				(triggerCentral.triggerState.someSortOfTriggerError() && !engineConfiguration->silentTriggerError)) {
				efiPrintf(
						"error: synchronizationPoint @ index %lu expected %d/%d got %d/%d",
						triggerCentral.triggerState.currentCycle.current_index,
						triggerCentral.triggerShape.getExpectedEventCount(TriggerWheel::T_PRIMARY),
						triggerCentral.triggerShape.getExpectedEventCount(TriggerWheel::T_SECONDARY),
						triggerCentral.triggerState.currentCycle.eventCount[0],
						triggerCentral.triggerState.currentCycle.eventCount[1]);
			}
#endif /* EFI_PROD_CODE */
		}

		engine->triggerCentral.triggerErrorDetection.add(isDecodingError);
	}
}
#endif

void Engine::injectEngineReferences() {
#if EFI_SHAFT_POSITION_INPUT
	triggerCentral.primaryTriggerConfiguration.update();
	for (int camIndex = 0; camIndex < CAMS_PER_BANK; camIndex++) {
		triggerCentral.vvtTriggerConfiguration[camIndex].update();
	}
#endif // EFI_SHAFT_POSITION_INPUT
}

void Engine::setConfig() {
	efi::clear(config);

	injectEngineReferences();
}

/* ===== ECU看门狗 =====
 * 安全机制: 如果发动机停止转动,自动关闭所有喷油器和点火线圈
 * 防止: 意外喷油(淹缸/液压锁)/线圈持续通电(过热损坏)
 *
 * 触发条件:
 * 1. not engineMovedRecently() → 超过阈值时间无触发信号
 * 2. 不在PWM测试模式中
 * 3. 不在起动预注油(priming)阶段
 *
 * 两遍通过机制:
 * 第一次: isSpinningJustForWatchdog=false → stopPins()但可能不完全关闭
 * 第二次(下一周期): 确认关闭,打印警告
 * 这是为了避免误触发(如触发信号短暂丢失)
 */
void Engine::efiWatchdog() {
#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT
	if (isRunningPwmTest) {
		return;
	}

	/* 注油模式: 这是起动前的燃油预注油阶段
	 * 发动机虽然没转但喷油器应该工作(建立油压/湿润气缸)
	 * 此时不触发看门狗
	 */
	if (module<PrimeController>()->isPriming()) {
		return;
	}

	if (!getTriggerCentral()->isSpinningJustForWatchdog) {
		/* 第一遍: 如果stopPins()返回true(说明有引脚被关闭了)
		 * 打印警告,但不触发固件错误(可能是瞬时信号丢失)
		 */
		if (!isRunningBenchTest() && enginePins.stopPins()) {
			warning(ObdCode::CUSTOM_ERR_2ND_WATCHDOG, "Some pins were turned off by 2nd pass watchdog");
		}
		return;
	}

	if (engine->triggerCentral.engineMovedRecently()) {
		// Engine moved recently, no need to safe pins.
		return;
	}

	/* 确认发动机已停止:
	 * 清除旋转标志,使点火调度无效
	 * 调用stopPins()关闭所有安全关键输出
	 */
	getTriggerCentral()->isSpinningJustForWatchdog = false;
	ignitionEvents.isReady = false;

	efiPrintf("Engine stopped, safing pins");

	enginePins.stopPins();
#endif // EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT
}

void Engine::checkShutdown() {
#if EFI_MAIN_RELAY_CONTROL
	// if we are already in the "ignition_on" mode, then do nothing
	/* this logic is not alive
		if (ignitionOnTimeNt > 0) {
			return;
		}
	todo: move to shutdown_controller.cpp
	*/

	// here we are in the shutdown (the ignition is off) or initial mode (after the firmware fresh start)
	// const efitick_t engineStopWaitTimeoutUs = 500000LL;	// 0.5 sec
	// in shutdown mode, we need a small cooldown time between the ignition off and on
/* this needs work or tests
todo: move to shutdown_controller.cpp
	if (stopEngineRequestTimeNt == 0 || (getTimeNowNt() - stopEngineRequestTimeNt) > US2NT(engineStopWaitTimeoutUs)) {
		// if the ignition key is turned on again,
		// we cancel the shutdown mode, but only if all shutdown procedures are complete
		const float vBattThresholdOn = 8.0f;
		// we fallback into zero instead of VBAT_FALLBACK_VALUE because it's not safe to false-trigger the "ignition on"
event,
		// and we want to turn on the main relay only when 100% sure.
		if ((Sensor::getOrZero(SensorType::BatteryVoltage) > vBattThresholdOn) && !isInShutdownMode()) {
			ignitionOnTimeNt = getTimeNowNt();
			efiPrintf("Ignition voltage detected!");
			if (stopEngineRequestTimeNt != 0) {
				efiPrintf("Cancel the engine shutdown!");
				stopEngineRequestTimeNt = 0;
			}
		}
	}
*/
#endif /* EFI_MAIN_RELAY_CONTROL */
}

bool Engine::isInShutdownMode() const {
	// TODO: this logic is currently broken
#if 0 && EFI_MAIN_RELAY_CONTROL && EFI_PROD_CODE
	// if we are in "ignition_on" mode and not in shutdown mode
	if (stopEngineRequestTimeNt == 0 && ignitionOnTimeNt > 0) {
		const float vBattThresholdOff = 5.0f;
		// start the shutdown process if the ignition voltage dropped low
		if (Sensor::get(SensorType::BatteryVoltage).value_or(VBAT_FALLBACK_VALUE) <= vBattThresholdOff) {
			scheduleStopEngine();
		}
	}

	// we are not in the shutdown mode?
	if (stopEngineRequestTimeNt == 0) {
		return false;
	}

	const efitick_t turnOffWaitTimeoutNt = NT_PER_SECOND;
	// We don't want any transients to step in, so we wait at least 1 second whatever happens.
	// Also it's good to give the stepper motor some time to start moving to the initial position (or parking)
	if ((getTimeNowNt() - stopEngineRequestTimeNt) < turnOffWaitTimeoutNt)
		return true;

	const efitick_t engineSpinningWaitTimeoutNt = 5 * NT_PER_SECOND;
	// The engine is still spinning! Give it some time to stop (but wait no more than 5 secs)
	if (isSpinning && (getTimeNowNt() - stopEngineRequestTimeNt) < engineSpinningWaitTimeoutNt)
		return true;

	// The idle motor valve is still moving! Give it some time to park (but wait no more than 10 secs)
	// Usually it can move to the initial 'cranking' position or zero 'parking' position.
	const efitick_t idleMotorWaitTimeoutNt = 10 * NT_PER_SECOND;
	if (isIdleMotorBusy() && (getTimeNowNt() - stopEngineRequestTimeNt) < idleMotorWaitTimeoutNt)
		return true;
#endif /* EFI_MAIN_RELAY_CONTROL */
	return false;
}

bool Engine::isMainRelayEnabled() const {
#if EFI_MAIN_RELAY_CONTROL
	return enginePins.mainRelay.getLogicValue();
#else
	// if no main relay control, we assume it's always turned on
	return true;
#endif /* EFI_MAIN_RELAY_CONTROL */
}

injection_mode_e getCurrentInjectionMode() {
	if (getEngineRotationState()->isCranking()) {
		return engineConfiguration->crankingInjectionMode;
	}

	auto runningMode = engineConfiguration->injectionMode;

#if EFI_SHAFT_POSITION_INPUT
	if (runningMode == IM_SEQUENTIAL) {
		bool missingPhaseInfoForSequential = !engine->triggerCentral.triggerState.hasSynchronizedPhase();

		bool willGetSequentialInfoLater = engine->triggerCentral.triggerState.expectDisambiguation();

		// IF
		// - We do not currently have full sync
		// - AND we expect to get it later (ie, once the cam syncs)
		// THEN hold off on sequential, and stay in batch fueling for now
		if (missingPhaseInfoForSequential && willGetSequentialInfoLater) {
			return IM_BATCH;
		}
	}
#endif /* EFI_SHAFT_POSITION_INPUT */

	return runningMode;
}

/**
 * The idea of this method is to execute all heavy calculations in a lower-priority thread,
 * so that trigger event handler/IO scheduler tasks are faster.
 */
void Engine::periodicFastCallback() {
	ScopePerf pc(PE::EnginePeriodicFastCallback);

	engineState.periodicFastCallback();

	speedoUpdate();

	engineModules.apply_all([](auto& m) { m.onFastCallback(); });
}

EngineRotationState* getEngineRotationState() {
	return &engine->rpmCalculator;
}

EngineState* getEngineState() {
	return &engine->engineState;
}

TunerStudioOutputChannels* getTunerStudioOutputChannels() {
	return &engine->outputChannels;
}

Scheduler* getScheduler() {
	return &engine->scheduler;
}

#if EFI_SHAFT_POSITION_INPUT
TriggerCentral* getTriggerCentral() {
	return &engine->triggerCentral;
}
#endif // EFI_SHAFT_POSITION_INPUT

LimpManager* getLimpManager() {
	return &engine->module<LimpManager>().unmock();
}

#if EFI_ENGINE_CONTROL
FuelSchedule* getFuelSchedule() {
	return &engine->injectionEvents;
}

IgnitionEventList* getIgnitionEvents() {
	return &engine->ignitionEvents;
}
#endif // EFI_ENGINE_CONTROL
