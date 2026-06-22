/*
 * @file spark_logic.cpp
 *
 * @date Sep 15, 2016
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#include "spark_logic.h"

#include "event_queue.h"

#include "knock_logic.h"

#if EFI_ENGINE_CONTROL

/**
 * @param cylinderIndex from 0 to cylinderCount, not cylinder number
 */
/* 根据点火模式将气缸索引映射到物理点火输出引脚:
 * ONE_COIL(分电器):    所有气缸共用一个线圈
 * WASTED_SPARK(双缸):  每两个气缸共用一个线圈(一个压缩,一个排气)
 * INDIVIDUAL_COILS(独立): 每个气缸一个线圈(COP)
 * TWO_COILS:            每两个气缸一个线圈(用于偶数缸+特殊配置)
 *
 * 返回: 线圈引脚索引(从0开始)
 */
static int getIgnitionPinForIndex(int cylinderIndex, ignition_mode_e ignitionMode) {
	switch (ignitionMode) {
		case IM_ONE_COIL:
			return 0;
		case IM_WASTED_SPARK: {
			if (engineConfiguration->cylindersCount == 1) {
				// we do not want to divide by zero
				return 0;
			}
			return cylinderIndex % (engineConfiguration->cylindersCount / 2);
		}
		case IM_INDIVIDUAL_COILS:
			return cylinderIndex;
		case IM_TWO_COILS:
			return cylinderIndex % 2;

		default:
			firmwareError(
					ObdCode::CUSTOM_OBD_IGNITION_MODE,
					"Invalid ignition mode getIgnitionPinForIndex(): %d",
					engineConfiguration->ignitionMode);
			return 0;
	}
}

/* ===== 单缸点火提前角计算 =====
 * m_timingAdvance: 来自3D表的基础点火提前角(°BTDC)
 * lateAdjustment: 所有"晚期修正"的总和,包括:
 *   - 爆震推迟(knockRetard,负值)
 *   - 怠速定时调整(正/负)
 *   - 弹射/ALS点火限制
 *
 * 输出钳位: 确保在[minimumIgnitionTiming, maximumIgnitionTiming]范围内
 * 典型值: -10°(TDC后, 非常迟) ~ 45°(TDC前, 非常早)
 *
 * 返回值: 调度的角度(°ATDC, 正值)
 * 注意: 计算值是°BTDC(上止点前), 调度器需要°ATDC(上止点后)
 * 所以取负号 + 气缸偏移量
 */
angle_t OneCylinder::getSparkAngle(angle_t lateAdjustment) const {
	// Compute the final ignition timing including all "late" adjustments
	angle_t finalIgnitionTiming = m_timingAdvance + lateAdjustment;

	// 10 ATDC ends up as 710, convert it to -10 so we can log and clamp correctly
	if (finalIgnitionTiming > 360) {
		finalIgnitionTiming -= 720;
	}

	// Clamp the final ignition timing to the configured limits
	// finalIgnitionTiming is deg BTDC
	// minimumIgnitionTiming limits maximium retard
	// maximumIgnitionTiming limits maximum advance
	finalIgnitionTiming =
			clampF(engineConfiguration->minimumIgnitionTiming,
				   finalIgnitionTiming,
				   engineConfiguration->maximumIgnitionTiming);

	engine->outputChannels.ignitionAdvanceCyl[m_cylinderNumber] = finalIgnitionTiming;

	return
			// Negate because timing *before* TDC, and we schedule *after* TDC
			-finalIgnitionTiming
			// Offset by this cylinder's position in the cycle
			+ getAngleOffset();
}

uint16_t IgnitionEvent::calculateIgnitionOutputMask() const {
	const int index = getIgnitionPinForIndex(cylinderIndex, m_ignitionMode);
	const int coilIndex = getCylinderNumberAtIndex(index);

	uint16_t outputsMask = 1 << coilIndex;

	// If wasted spark, find the paired coil in addition to "main" output for this cylinder.
	// Skip in pairedOddFireWastedSpark mode: a single physical coil is shared between companion cylinders, so only the
	// first-half (lower firing-order index) cylinder slot gets a pin assigned and we drive only that one OutputPin from
	// both events.
	if (m_ignitionMode == IM_WASTED_SPARK && !engineConfiguration->pairedOddFireWastedSpark) {
		int secondIndex = index + engineConfiguration->cylindersCount / 2;
		int secondCoilIndex = getCylinderNumberAtIndex(secondIndex);
		outputsMask |= 1 << secondCoilIndex;
	}

	return outputsMask;
}

angle_t IgnitionEvent::calculateSparkAngle() const {
	angle_t sparkAngle = engine->cylinders[cylinderNumber].getSparkAngle(
			// Pull any extra timing for knock retard
			-engine->module<KnockController>()->getKnockRetard());

	efiAssert(ObdCode::CUSTOM_SPARK_ANGLE_1, !std::isnan(sparkAngle), "sparkAngle#1", 0);
	wrapAngle(sparkAngle, "findAngle#2", ObdCode::CUSTOM_ERR_6550);

	return sparkAngle;
}

static void prepareCylinderIgnitionSchedule(angle_t dwellAngleDuration, floatms_t sparkDwell, IgnitionEvent& event) {
	// todo: clean up this implementation? does not look too nice as is.

	const int realCylinderNumber = getCylinderNumberAtIndex(event.cylinderIndex);

	// let's save planned duration so that we can later compare it with reality
	event.sparkDwell = sparkDwell;

	// Stash which cylinder we're scheduling so that knock sensing knows which
	// cylinder just fired
	event.cylinderNumber = realCylinderNumber;

	auto sparkAngle = event.calculateSparkAngle();

	auto ignitionMode = getCurrentIgnitionMode();

	// On an odd cylinder (or odd fire) wasted spark engine, map outputs as if in sequential.
	// During actual scheduling, the events just get scheduled every 360 deg instead
	// of every 720 deg.
	if (ignitionMode == IM_WASTED_SPARK && engine->engineState.useOddFireWastedSpark) {
		ignitionMode = IM_INDIVIDUAL_COILS;
	}

	angle_t dwellStartAngle = sparkAngle - dwellAngleDuration;
	efiAssertVoid(ObdCode::CUSTOM_ERR_6590, !std::isnan(dwellStartAngle), "findAngle#5");

	assertAngleRange(dwellStartAngle, "findAngle dwellStartAngle", ObdCode::CUSTOM_ERR_6550);
	wrapAngle(dwellStartAngle, "findAngle#7", ObdCode::CUSTOM_ERR_6550);

	event.m_ignitionMode = ignitionMode;
	event.dwellAngle = dwellStartAngle;

	engine->outputChannels.currentIgnitionMode = static_cast<uint8_t>(ignitionMode);
}

static void chargeTrailingSpark(IgnitionOutputPin* pin) {
	pin->setHigh();
}

static void fireTrailingSpark(IgnitionOutputPin* pin) {
	pin->setLow();
}

/* ===== 火花触发 + 下次调度准备 =====
 * 这是点火系统的核心回调函数,当点火时间到达时被调度器调用
 * ctx中包含: 输出掩码、事件索引、剩余多火花次数、过充保护标志
 *
 * 功能:
 * 1. Underdwell保护: 如果实际dwell时间不足,推迟点火(含推迟提前角)
 * 2. 关闭线圈(产生火花)
 * 3. 如果有多火花需求,调度下一个dwell+火花
 * 4. 如果有trailing spark需求,在指定角度调度
 * 5. 准备好下一个周期的点火调度
 *
 * 设计哲学: "宁可推迟点火也不失火"
 * 推迟点火损失功率,但失火会导致排放超标和动力中断
 */
void fireSparkAndPrepareNextSchedule(IgnitionContext ctx) {
	efitick_t nowNt = getTimeNowNt();
	auto& event = engine->ignitionEvents.elements[ctx.eventIndex];

	/* Underdwell检测:
	 * 如果实际线圈充电时间 < 目标dwell的80%
	 * 说明转速突然升高导致dwell时间不够
	 * 解决方案: 推迟点火(减小提前角)让线圈多充一会儿
	 * 实际是推迟火花触发时间 → 等效于推迟点火提前角
	 */
	float actualDwellMs = event.actualDwellTimer.getElapsedSeconds(nowNt) * 1e3;
	float minDwell = 0.8f * event.sparkDwell;
	if (!ctx.isOverdwellProtect && actualDwellMs < minDwell) {
		float extraTimeUs = (minDwell - actualDwellMs) * 1e3;

		if (extraTimeUs < 10) {
			extraTimeUs = 10;
		}

		efitick_t delayedFireTime = nowNt + efidur_t{(uint32_t)USF2NT(extraTimeUs)};

		// cancel multispark in case of underdwell
		ctx.sparksRemaining = 0;

		// re-schedule ourselves at a later time once enough dwell has elapsed
		// This is fine to do because it will retard the effective ignition timing, but
		// ensure the coil has enough energy to actually fire (we would rather retard timing than misfire)
		engine->scheduler.schedule(
				"firing", &event.sparkEvent.scheduling, delayedFireTime, {fireSparkAndPrepareNextSchedule, ctx});
		return;
	}

#if EFI_UNIT_TEST
	if (engine->onIgnitionEvent) {
		engine->onIgnitionEvent(ctx, false);
	}
#endif

	/* 关闭线圈: setLow()使线圈初级电流中断
	 * 电磁场崩溃 → 次级产生高压 → 火花塞跳火
	 * forEachSetBit: 遍历输出掩码中所有位,逐个关闭
	 * 在wasted spark模式下,一个事件可能控制两个线圈
	 */
	forEachSetBit(ctx.outputsMask, [](size_t idx) { enginePins.coils[idx].setLow(); });

#if EFI_TUNER_STUDIO
	// ratio of desired dwell duration to actual dwell duration gives us some idea of how good is input trigger jitter
	engine->outputChannels.dwellAccuracyRatio = actualDwellMs / event.sparkDwell;
#endif

	// now that we've just fired a coil let's prepare the new schedule for the next engine revolution

	angle_t dwellAngleDuration = engine->ignitionState.dwellAngle;
	floatms_t sparkDwell = engine->ignitionState.getDwell();
	if (std::isnan(dwellAngleDuration) || std::isnan(sparkDwell)) {
		// we are here if engine has just stopped
		return;
	}

	/* 多火花: 如果还有剩余火花次数
	 * 稀薄燃烧或寒冷天气时需要多个火花确保点燃
	 * 额外火花 ASAP(尽快)触发
	 */
	if (ctx.sparksRemaining > 0 && !ctx.isOverdwellProtect) {
		ctx.sparksRemaining--;

		efitick_t nextDwellStart = nowNt + engine->engineState.multispark.delay;
		efitick_t nextFiring = nextDwellStart + engine->engineState.multispark.dwell;

		// We can schedule both of these right away, since we're going for "asap" not "particular angle"
		engine->scheduler.schedule("dwell", &event.dwellStartTimer, nextDwellStart, {&turnSparkPinHigh, ctx});
		engine->scheduler.schedule(
				"firing", &event.sparkEvent.scheduling, nextFiring, {fireSparkAndPrepareNextSchedule, ctx});
	} else {
		/* 如果启用了Trailing Spark(高能点火):
		 * 在主火花之后延迟一定角度,再产生一个较弱火花
		 * 用于稀薄燃烧或高EGR时辅助点燃
		 */
		if (engineConfiguration->enableTrailingSparks && !ctx.isOverdwellProtect) {
			// Trailing sparks are enabled - schedule an event for the corresponding trailing coil
			scheduleByAngle(
					&event.trailingSparkFire,
					nowNt,
					engine->engineState.trailingSparkAngle,
					{&fireTrailingSpark, &enginePins.trailingCoils[event.cylinderNumber]});
		}

		/* 重新准备本缸的下一个周期点火调度
		 * dwell角度可能因运行条件变化而不同
		 */
		prepareCylinderIgnitionSchedule(dwellAngleDuration, sparkDwell, event);
	}

	/* 通知爆震系统刚刚哪个气缸点火了
	 * 爆震控制器用此信息在正确的时间窗口开始采样
	 */
	engine->onSparkFireKnockSense(event.cylinderNumber);
}

void turnSparkPinHigh(IgnitionContext ctx) {
	efitick_t nowNt = getTimeNowNt();

	forEachSetBit(ctx.outputsMask, [](size_t idx) { enginePins.coils[idx].setHigh(); });

	auto& event = engine->ignitionEvents.elements[ctx.eventIndex];

	event.actualDwellTimer.reset(nowNt);

#if EFI_UNIT_TEST
	if (engine->onIgnitionEvent) {
		engine->onIgnitionEvent(ctx, true);
	}
#endif

	if (engineConfiguration->enableTrailingSparks) {
		IgnitionOutputPin* output = &enginePins.trailingCoils[event.cylinderNumber];
		// Trailing sparks are enabled - schedule an event for the corresponding trailing coil
		scheduleByAngle(
				&event.trailingSparkCharge,
				nowNt,
				engine->engineState.trailingSparkAngle,
				{&chargeTrailingSpark, output});
	}
}

static void scheduleSparkEvent(
		bool limitedSpark,
		IgnitionEvent& event,
		float dwellMs,
		EngPhase dwellAngle,
		EngPhase sparkAngle,
		const EnginePhaseInfo& phase) {
	float angleOffset = dwellAngle - phase.currentEngPhase;
	if (angleOffset < 0) {
		angleOffset += engine->engineState.engineCycle;
	}

	engine->engineState.sparkCounter++;
	event.wasSparkLimited = limitedSpark;

	IgnitionContext ctx;
	ctx.outputsMask = event.calculateIgnitionOutputMask();
	ctx.eventIndex = event.cylinderIndex;
	ctx.sparksRemaining = limitedSpark ? 0 : engine->engineState.multispark.count;

	efitick_t chargeTime;

	/**
	 * The start of charge is always within the current trigger event range, so just plain time-based scheduling
	 */
	if (!limitedSpark) {
		/**
		 * Note how we do not check if spark is limited or not while scheduling 'spark down'
		 * This way we make sure that coil dwell started while spark was enabled would fire and not burn
		 * the coil.
		 */
		chargeTime = scheduleByAngle(&event.dwellStartTimer, phase.timestamp, angleOffset, {&turnSparkPinHigh, ctx});
	}

	/**
	 * Spark event is often happening during a later trigger event timeframe
	 */

	efiAssertVoid(ObdCode::CUSTOM_ERR_6591, !std::isnan(sparkAngle.angle), "findAngle#4");
	assertAngleRange(sparkAngle.angle, "findAngle#a5", ObdCode::CUSTOM_ERR_6549);

	bool scheduled = engine->module<TriggerScheduler>()->scheduleOrQueue(
			&event.sparkEvent, sparkAngle, {fireSparkAndPrepareNextSchedule, ctx}, phase);

	if (!scheduled && !limitedSpark) {
		// If spark firing wasn't already scheduled, schedule the overdwell event at
		// 1.5x nominal dwell, should the trigger disappear before its scheduled for real
		efitick_t fireTime = chargeTime + (uint32_t)MSF2NT(1.5f * dwellMs);
		ctx.isOverdwellProtect = true;
		engine->scheduler.schedule(
				"overdwell", &event.sparkEvent.scheduling, fireTime, {fireSparkAndPrepareNextSchedule, ctx});
	}
}

void initializeIgnitionActions() {
	IgnitionEventList& list = engine->ignitionEvents;
	angle_t dwellAngle = engine->ignitionState.dwellAngle;
	floatms_t sparkDwell = engine->ignitionState.getDwell();
	if (std::isnan(engine->cylinders[0].getIgnitionTimingBtdc()) || std::isnan(dwellAngle)) {
		// error should already be reported
		// need to invalidate previous ignition schedule
		list.isReady = false;
		return;
	}
	efiAssertVoid(ObdCode::CUSTOM_ERR_6592, engineConfiguration->cylindersCount > 0, "cylindersCount");

	for (size_t cylinderIndex = 0; cylinderIndex < engineConfiguration->cylindersCount; cylinderIndex++) {
		list.elements[cylinderIndex].cylinderIndex = cylinderIndex;
		prepareCylinderIgnitionSchedule(dwellAngle, sparkDwell, list.elements[cylinderIndex]);
	}
	list.isReady = true;
}

static void prepareIgnitionSchedule() {
	ScopePerf perf(PE::PrepareIgnitionSchedule);

	/**
	 * TODO: warning. there is a bit of a hack here, todo: improve.
	 * currently output signals/times dwellStartTimer from the previous revolutions could be
	 * still used because they have crossed the revolution boundary
	 * but we are already re-purposing the output signals, but everything works because we
	 * are not affecting that space in memory. todo: use two instances of 'ignitionSignals'
	 */
	operation_mode_e operationMode = getEngineRotationState()->getOperationMode();
	float maxAllowedDwellAngle = (int)(getEngineCycle(operationMode) / 2); // the cast is about making Coverity happy

	if (getCurrentIgnitionMode() == IM_ONE_COIL) {
		maxAllowedDwellAngle = getEngineCycle(operationMode) / engineConfiguration->cylindersCount / 1.1;
	}

	if (engine->ignitionState.dwellAngle == 0) {
		warning(ObdCode::CUSTOM_ZERO_DWELL, "dwell is zero?");
	}
	if (engine->ignitionState.dwellAngle > maxAllowedDwellAngle) {
		warning(ObdCode::CUSTOM_DWELL_TOO_LONG, "dwell angle too long: %.2f", engine->ignitionState.dwellAngle);
	}

	// todo: add some check for dwell overflow? like 4 times 6 ms while engine cycle is less then that

	initializeIgnitionActions();
}

/* ===== 触发事件中的点火调度入口 =====
 * 每次触发齿事件(frame)调用,检查是否有需要开始dwell的气缸
 * 由于dwell开始时间可能跨多个frame,这里检查dwell角度是否在当前frame内
 *
 * 对每个气缸:
 * 1. 检查dwell起始角是否在当前的触发相位窗口内
 * 2. 处理奇数缸浪费点火(wasted spark)的特殊情况
 * 3. 应用弹射/扭矩限制/ALS的火花跳过逻�b
 * 4. 调度dwell开始和火花触发
 *
 * limitedSpark: 由limp_manager控制,限制点火能力
 *   (爆震/超温/超压等保护场景)
 */
void onTriggerEventSparkLogic(const EnginePhaseInfo& phase) {
	ScopePerf perf(PE::OnTriggerEventSparkLogic);

	if (!engineConfiguration->isIgnitionEnabled) {
		return;
	}

	bool limitedSpark = !getLimpManager()->allowIgnition().value;

	const floatms_t dwellMs = engine->ignitionState.getDwell();
	if (std::isnan(dwellMs) || dwellMs <= 0) {
		warning(ObdCode::CUSTOM_DWELL, "invalid dwell to handle: %.2f", dwellMs);
		return;
	}

	if (!engine->ignitionEvents.isReady) {
		prepareIgnitionSchedule();
	}

	/**
	 * Ignition schedule is defined once per revolution
	 * See initializeIgnitionActions()
	 */

	// Only apply odd cylinder count wasted logic if:
	// - odd cyl count
	// - current mode is wasted spark
	// - four stroke
	bool enableOddCylinderWastedSpark =
			engine->engineState.useOddFireWastedSpark && getCurrentIgnitionMode() == IM_WASTED_SPARK;

	if (engine->ignitionEvents.isReady) {
		for (size_t i = 0; i < engineConfiguration->cylindersCount; i++) {
			auto& event = engine->ignitionEvents.elements[i];

			angle_t dwellAngle = event.dwellAngle;

			angle_t sparkAngleAdjust = 0;

			/* 奇数缸浪费点火处理:
			 * 对于奇数缸发动机(如3缸/5缸),传统的Wasted Spark在360°间隔的火花
			 * 不能简单配对。这里检查360°之后的dwell事件是否在当前相位窗口,
			 * 如果是则立即处理(而不是等到360°后)
			 */
			bool isOddCylWastedEvent = false;
			if (enableOddCylinderWastedSpark) {
				auto dwellAngleWastedEvent = dwellAngle + 360;
				if (dwellAngleWastedEvent > 720) {
					dwellAngleWastedEvent -= 720;
				}

				// Check whether this event hits 360 degrees out from now (ie, wasted spark),
				// and if so, twiddle the dwell and spark angles so it happens now instead
				isOddCylWastedEvent = isPhaseInRange(EngPhase{dwellAngleWastedEvent}, phase);

				if (isOddCylWastedEvent) {
					dwellAngle = dwellAngleWastedEvent;

					sparkAngleAdjust = 360;
				}
			}

			/* dwell起始角不在当前相位窗口 → 跳过,等下次触发事件再检查 */
			if (!isOddCylWastedEvent && !isPhaseInRange(EngPhase{dwellAngle}, phase)) {
				continue;
			}

			angle_t sparkAngle = sparkAngleAdjust + event.calculateSparkAngle();
			if (sparkAngle > 720) {
				sparkAngle -= 720;
			}
			if (std::isnan(sparkAngle)) {
				warning(ObdCode::CUSTOM_ADVANCE_SPARK, "NaN advance");
				continue;
			}

			/* 弹射起步/扭矩降低: shouldSkip()根据跳过比率随机跳过火花
			 * 这会产生受控的"断火"来限制发动机扭矩
			 * 弹射时控制起步加速度,扭矩降低用于换挡辅助
			 */
#if EFI_LAUNCH_CONTROL
			if (engine->softSparkLimiter.shouldSkip()) {
				continue;
			}

			if (engine->torqueReductionSparkLimiter.shouldSkip()) {
				continue;
			}
#endif // EFI_LAUNCH_CONTROL

			/* ALS(防滞后系统): 跳过部分火花产生更热的排气
			 * 驱动涡轮在松油门时保持转速
			 * ALSSkipRatio控制跳过比例(如跳过50%的火花)
			 */
#if EFI_ANTILAG_SYSTEM && EFI_LAUNCH_CONTROL
			if (engine->ALSsoftSparkLimiter.shouldSkip()) {
				continue;
			}
			auto ALSSkipRatio = engineConfiguration->ALSSkipRatio;
			engine->ALSsoftSparkLimiter.setTargetSkipRatio(ALSSkipRatio);
#endif // EFI_ANTILAG_SYSTEM

			scheduleSparkEvent(limitedSpark, event, dwellMs, {dwellAngle}, {sparkAngle}, phase);
		}
	}
}

/**
 * Number of sparks per physical coil
 * @see getNumberOfInjections
 */
int getNumberOfSparks(ignition_mode_e mode) {
	switch (mode) {
		case IM_ONE_COIL:
			return engineConfiguration->cylindersCount;
		case IM_TWO_COILS:
			return engineConfiguration->cylindersCount / 2;
		case IM_INDIVIDUAL_COILS:
			return 1;
		case IM_WASTED_SPARK:
			return 2;
		default:
			firmwareError(ObdCode::CUSTOM_ERR_IGNITION_MODE, "Unexpected ignition_mode_e %d", mode);
			return 1;
	}
}

/**
 * @see getInjectorDutyCycle
 */
percent_t getCoilDutyCycle(float rpm) {
	floatms_t totalPerCycle = engine->ignitionState.getDwell() * getNumberOfSparks(getCurrentIgnitionMode());
	floatms_t engineCycleDuration =
			getCrankshaftRevolutionTimeMs(rpm) * (getEngineRotationState()->getOperationMode() == TWO_STROKE ? 1 : 2);
	return 100 * totalPerCycle / engineCycleDuration;
}

#endif // EFI_ENGINE_CONTROL
