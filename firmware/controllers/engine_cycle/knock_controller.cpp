/*
 * @file knock_logic.c
 * [爆震控制]
 * 处理爆震传感器信号的判定和点火推迟响应。
 *
 * 爆震检测算法:
 * 1. 在每个气缸点火后的特定角度窗口采样
 * 2. 对原始ADC信号做带通滤波(5-15kHz,取决于缸径)
 * 3. 计算RMS值→dBV转换
 * 4. 每个气缸独立的基础噪声自适应
 * 5. 信号 > 基础噪声 + 配置阈值 → 判定为爆震
 *
 * 爆震响应:
 * 1. 立即推迟该气缸点火(knockRetardAggression控制退角速度)
 * 2. 无爆震时逐渐恢复到正常提前角
 * 3. 逐缸控制确保仅推迟爆震的气缸,最大化性能
 *
 * @date Apr 04, 2021
 * @author Andrey Gusakov
 */

#include "pch.h"
#include "knock_logic.h"

int getCylinderKnockBank(uint8_t cylinderNumber) {
	// C/C++ can't index in to bit fields, we have to provide lookup ourselves
	switch (cylinderNumber) {
		case 0:
			return engineConfiguration->knockBankCyl1;
		case 1:
			return engineConfiguration->knockBankCyl2;
		case 2:
			return engineConfiguration->knockBankCyl3;
		case 3:
			return engineConfiguration->knockBankCyl4;
		case 4:
			return engineConfiguration->knockBankCyl5;
		case 5:
			return engineConfiguration->knockBankCyl6;
		case 6:
			return engineConfiguration->knockBankCyl7;
		case 7:
			return engineConfiguration->knockBankCyl8;
		case 8:
			return engineConfiguration->knockBankCyl9;
		case 9:
			return engineConfiguration->knockBankCyl10;
		case 10:
			return engineConfiguration->knockBankCyl11;
		case 11:
			return engineConfiguration->knockBankCyl12;
		default:
			return 0;
	}
}

bool KnockControllerBase::onKnockSenseCompleted(
		uint8_t cylinderNumber, uint8_t channelIdx, float dbv, efitick_t lastKnockTime) {
	// Adjust by the user-configured gain for this cylinder
	dbv += m_gain[cylinderNumber];

	bool isKnock = dbv > m_knockThreshold;

	// Per-cylinder peak detector
	float cylPeak = peakDetectors[cylinderNumber].detect(dbv, lastKnockTime);
	m_knockCyl[cylinderNumber] = roundf(cylPeak);

	// All-cylinders peak detector
	m_knockLevel = allCylinderPeakDetector.detect(dbv, lastKnockTime);

	if (isKnock) {
		m_knockCount++;
		m_lastKnockTimer.reset(lastKnockTime);

		auto baseTiming = engine->cylinders[cylinderNumber].getIgnitionTimingBtdc();

		// TODO: 20 configurable? Better explanation why 20?
		auto distToMinimum = baseTiming - (-20);

		// percent -> ratio = divide by 100
		auto retardFraction = engineConfiguration->knockRetardAggression * 0.01f;
		auto retardAmount = distToMinimum * retardFraction;

		{
			// Adjust knock retard under lock
			chibios_rt::CriticalSectionLocker csl;
			auto newRetard = m_knockRetard + retardAmount;
			m_knockRetard = clampF(0, newRetard, m_maximumRetard);
		}
	}

	engine->module<SensorChecker>()->onKnockSensorSignal(dbv, channelIdx, lastKnockTime);

	return isKnock;
}

float KnockControllerBase::getKnockRetard() const {
	return m_knockRetard;
}

uint32_t KnockControllerBase::getKnockCount() const {
	return m_knockCount;
}

void KnockControllerBase::onFastCallback() {
	constexpr auto callbackPeriodSeconds = FAST_CALLBACK_PERIOD_MS / 1000.0f;

	auto applyAmount = engineConfiguration->knockRetardReapplyRate * callbackPeriodSeconds;

	{
		// Adjust knock retard under lock
		chibios_rt::CriticalSectionLocker csl;

		// Reduce knock retard at the requested rate
		float newRetard = m_knockRetard - applyAmount;

		// don't allow retard to go negative
		if (newRetard < 0) {
			m_knockRetard = 0;
		} else {
			m_knockRetard = newRetard;
		}
	}

	hasKnockRecently = !m_lastKnockTimer.hasElapsedSec(0.5f);
	hasKnockRetardNow = m_knockRetard > 0;

	m_knockThreshold = getKnockThreshold();
	m_maximumRetard = getMaximumRetard();

	auto rpm = Sensor::getOrZero(SensorType::Rpm);
	auto load = getIgnitionLoad();

	for (size_t i = 0; i < engineConfiguration->cylindersCount; i++) {
		m_gain[i] = interpolate3d(
				config->knockGains[i].table, config->knockGainLoadBins, load, config->knockGainRpmBins, rpm);
	}
}

float KnockController::getKnockThreshold() const {
	return interpolate2d(Sensor::getOrZero(SensorType::Rpm), config->knockNoiseRpmBins, config->knockBaseNoise);
}

float KnockController::getMaximumRetard() const {
	return interpolate3d(
			config->maxKnockRetardTable,
			config->maxKnockRetardLoadBins,
			getIgnitionLoad(),
			config->maxKnockRetardRpmBins,
			Sensor::getOrZero(SensorType::Rpm));
}

// This callback is to be implemented by the knock sense driver
__attribute__((weak)) void onStartKnockSampling(uint8_t cylinderNumber, float samplingTimeSeconds, uint8_t channelIdx) {
	UNUSED(cylinderNumber);
	UNUSED(samplingTimeSeconds);
	UNUSED(channelIdx);
}

static uint8_t cylinderNumberCopy;

// Called when its time to start listening for knock
// Does some math, then hands off to the driver to start any sampling hardware
void Engine::onSparkFireKnockSense(uint8_t cylinderNumber) {
	cylinderNumberCopy = cylinderNumber;

#if EFI_SOFTWARE_KNOCK
	if (!engine->rpmCalculator.isRunning()) {
		return;
	}

	// Convert sampling angle to time
	float samplingSeconds =
			engine->rpmCalculator.oneDegreeUs * engineConfiguration->knockSamplingDuration / US_PER_SECOND_F;

	// Look up which channel this cylinder uses
	auto channel = getCylinderKnockBank(cylinderNumberCopy);

	// Call the driver to begin sampling
	onStartKnockSampling(cylinderNumberCopy, samplingSeconds, channel);
#endif
}
