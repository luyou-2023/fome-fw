#include "pch.h"

#include "ac_control.h"
#include "deadband.h"

/* [空调控制] 在保护发动机的前提下控制空调压缩机离合器
 * 允许条件: RPM在范围内 + CLT正常 + TPS不高 + 有请求(按钮+压力开关)
 * 保护条件: RPM过低(熄火风险)/RPM过高/水温过高/大负荷
 * 使用滞回比较器(Deadband)防止频繁开关切换
 */
static Deadband<200> maxRpmDeadband;
static Deadband<5> maxCltDeadband;
static Deadband<5> maxTpsDeadband;

bool AcController::getAcState() {
	auto rpm = Sensor::getOrZero(SensorType::Rpm);

	engineTooSlow = rpm < 500;

	if (engineTooSlow) {
		return false;
	}

	auto maxRpm = engineConfiguration->maxAcRpm;
	engineTooFast = maxRpm != 0 && maxRpmDeadband.gt(rpm, maxRpm);
	if (engineTooFast) {
		return false;
	}

	auto clt = Sensor::get(SensorType::Clt);

	noClt = !clt;
	// No AC with failed CLT
	if (noClt) {
		return false;
	}

	// Engine too hot, disable
	auto maxClt = engineConfiguration->maxAcClt;
	engineTooHot = (maxClt != 0) && maxCltDeadband.gt(clt.Value, maxClt);
	if (engineTooHot) {
		return false;
	}

	// TPS too high, disable
	auto maxTps = engineConfiguration->maxAcTps;
	tpsTooHigh = maxTps != 0 && maxTpsDeadband.gt(Sensor::getOrZero(SensorType::Tps1), maxTps);
	if (tpsTooHigh) {
		return false;
	}

	if (isDisabledByLua) {
		return false;
	}

	if (hasAcPressure() && !getAcPressure()) {
		return false;
	}

#if EFI_SHAFT_POSITION_INPUT
	if (engine->rpmCalculator.getSecondsSinceEngineStart(getTimeNowNt()) < engineConfiguration->acStartDelay) {
		return false;
	}
#endif // EFI_SHAFT_POSITION_INPUT

	// All conditions allow AC, simply pass thru switch
	return acButtonState;
}

void AcController::onSlowCallback() {
	bool isEnabled = getAcState();

	m_acEnabled = isEnabled;

	if (!isEnabled) {
		// reset the timer if AC is off
		m_timeSinceNoAc.reset();
	}

	float acDelay = engineConfiguration->acDelay;
	if (acDelay == 0) {
		// Without delay configured, enable immediately
		acCompressorState = isEnabled;
	} else {
		acCompressorState = isEnabled && m_timeSinceNoAc.hasElapsedSec(acDelay);
	}

	enginePins.acRelay.setValue(acCompressorState);
}

bool AcController::isAcEnabled() const {
	return m_acEnabled;
}
