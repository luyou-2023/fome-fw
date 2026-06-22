#pragma once
/* ===== 主循环周期调度 =====
 * 主循环以1000Hz运行，通过位掩码将不同频率的任务分配到不同周期
 * 1000Hz: 每周期执行 (ADC快速通道/原始输入)
 * 500Hz:  每2周期执行 (ADC慢速通道/电子节气门)
 * 250Hz:  每4周期执行 (快速回调 - 燃油/点火计算)
 * 20Hz:   每50周期执行 (慢速回调 - 传感器/看门狗)
 * 这种分时调度避免了多个定时器线程的竞争和同步开销
 */

void initMainLoop();

enum class LoopPeriod : uint8_t {
	None = 0,
	Period1000hz = 1 << 0,  // 位0: 1000Hz任务
	Period500hz = 1 << 1,   // 位1: 500Hz任务
	Period250hz = 1 << 2,   // 位2: 250Hz任务
	Period20hz = 1 << 3,    // 位3: 20Hz任务
};

inline constexpr LoopPeriod& operator|=(LoopPeriod& a, const LoopPeriod& b) {
	a = static_cast<LoopPeriod>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
	return a;
}

inline constexpr bool operator&(LoopPeriod a, LoopPeriod b) {
	return 0 != (static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

constexpr int hzForPeriod(LoopPeriod p) {
	switch (p) {
		case LoopPeriod::None:
			return 0;
		case LoopPeriod::Period1000hz:
			return 1000;
		case LoopPeriod::Period500hz:
			return 500;
		case LoopPeriod::Period250hz:
			return 250;
		case LoopPeriod::Period20hz:
			return 20;
	}

	return 0;
}

constexpr float loopPeriodMs(LoopPeriod p) {
	return 1000.0f / hzForPeriod(p);
}

#ifndef ADC_UPDATE_RATE
#define ADC_UPDATE_RATE LoopPeriod::Period500hz
#endif

#define ETB_UPDATE_RATE LoopPeriod::Period500hz
#define FAST_CALLBACK_RATE LoopPeriod::Period250hz
#define SLOW_CALLBACK_RATE LoopPeriod::Period20hz

#define FAST_CALLBACK_PERIOD_MS loopPeriodMs(FAST_CALLBACK_RATE)
#define SLOW_CALLBACK_PERIOD_MS loopPeriodMs(SLOW_CALLBACK_RATE)
