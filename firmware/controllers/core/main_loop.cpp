#include "pch.h"

#include "periodic_thread_controller.h"
#include "electronic_throttle.h"

/* 主循环以1000Hz(1ms)周期运行，是ECU控制计算的核心调度器
 * 使用单线程+分时复用策略，避免多线程竞争和同步开销
 * 不同频率的任务通过周期计数器模运算分发到各周期执行
 */
#define MAIN_LOOP_RATE 1000

class MainLoop final : PeriodicController<1024> {
public:
	MainLoop();
	void PeriodicTask(efitick_t nowNt) override;

	void startMainLoop() {
		m_stallTimer.reset();
		startThread();
	}

private:
	/* 模板函数: 编译期计算指定频率的标志位
	 * 模板参数TFlag在编译时确定，无运行时开销
	 * 如Period500hz: 每2周期返回一次true(1000/500=2)
	 */
	template <LoopPeriod TFlag>
	LoopPeriod makePeriodFlag() const;

	/* 组合所有周期的标志位为位掩码 */
	LoopPeriod makePeriodFlags();

	/* 周期计数器: 0→999循环递增
	 * 用于模运算确定当前周期需要执行哪些任务
	 */
	int m_cycleCounter = 0;

	/* 停滞检测定时器: 如果主循环超过0.1s未运行(被高优先级中断阻塞)
	 * 打印警告信息用于调试
	 */
	Timer m_stallTimer;
};

/* CCM_OPTIONAL: 如果MCU有紧耦合内存(CCM)，将其放在CCM中
 * CCM访问延迟更低，适合频繁调用的主循环对象
 */
static MainLoop mainLoop CCM_OPTIONAL;

void initMainLoop() {
	mainLoop.startMainLoop();
}

/* Prio: PRIO_MAIN_LOOP - 中等优先级，低于中断处理
 * Rate: 1000Hz - 每1ms执行一次PeriodicTask
 */
MainLoop::MainLoop()
	: PeriodicController("MainLoop", PRIO_MAIN_LOOP, MAIN_LOOP_RATE) {}

/* 核心周期任务 - 每1ms执行一次
 * nowNt: ChibiOS系统节拍时间戳(纳秒级)
 * 不同频率任务按周期分发:
 *   500Hz(2ms): ADC更新 + 电子节气门
 *   250Hz(4ms): 快速回调(燃油/点火计算)
 *   20Hz(50ms): 慢速回调(传感器/看门狗)
 */
void MainLoop::PeriodicTask(efitick_t nowNt) {
	ScopePerf perf(PE::MainLoop);

	/* 停滞检测: 如果本次调用距上次超过0.1s
	 * 说明高优先级任务(如中断)占用了过多CPU时间
	 * 此时应检查中断频率是否异常
	 */
	auto elapsedSinceLastLoop = m_stallTimer.getElapsedSecondsAndReset(nowNt);
	if (elapsedSinceLastLoop > 0.1) {
		efiPrintf("Main loop stall of %.3f sec detected", elapsedSinceLastLoop);
	}

	LoopPeriod p = makePeriodFlags();

#if HAL_USE_ADC
	/* 500Hz: 读取慢速ADC通道(水温/进气温度/电池电压等)
	 * 慢速ADC不需要高速采样，500Hz足以满足精度需求
	 */
	if (p & ADC_UPDATE_RATE) {
		updateSlowAdc(nowNt);
	}
#endif // HAL_USE_ADC

#if EFI_ELECTRONIC_THROTTLE_BODY
	/* 500Hz: 电子节气门位置闭环控制
	 * 节气门控制需要较高的更新率以确保响应及时
	 * 逐个更新所有ETB控制器(支持双节气门配置)
	 */
	if (p & ETB_UPDATE_RATE) {
		for (int i = 0; i < ETB_COUNT; i++) {
			auto etb = engine->etbControllers[i];

			if (etb) {
				etb->update();
			}
		}
	}
#endif // EFI_ELECTRONIC_THROTTLE_BODY

	/* 20Hz(50ms): 慢速周期任务
	 * 包括: 传感器状态检查、配置变更检测、看门狗、TPS加速补偿
	 * 这些任务不需要高频率执行，放在低频可以节省CPU
	 */
	if (p & SLOW_CALLBACK_RATE) {
		doPeriodicSlowCallback();
	}

	/* 250Hz(4ms): 快速周期任务
	 * 包括: 燃油量计算、点火提前角计算、怠速控制
	 * 这些是ECU的核心控制计算，需要较高频率但不必每个周期都算
	 * 4ms的计算延迟对发动机控制是完全可以接受的
	 */
	if (p & FAST_CALLBACK_RATE) {
		engine->periodicFastCallback();
	}
}

/* 编译期常量化: 计算指定频率的周期数
 * 如Period500hz → MAIN_LOOP_RATE(1000) / 500 = 2
 * static_assert确保整除，否则编译报错
 * constexpr保证编译期计算，无运行时开销
 */
template <LoopPeriod flag>
static constexpr int loopCounts() {
	constexpr auto hz = hzForPeriod(flag);

	// check that this cleanly divides
	static_assert(MAIN_LOOP_RATE % hz == 0);

	return MAIN_LOOP_RATE / hz;
}

/* 判断当前周期是否为指定频率的执行周期
 * 使用模运算: 如500Hz(loopCounts=2) → 每偶数周期执行
 * 结果: 是→返回标志位; 否→返回None
 */
template <LoopPeriod TFlag>
LoopPeriod MainLoop::makePeriodFlag() const {
	if (m_cycleCounter % loopCounts<TFlag>() == 0) {
		return TFlag;
	} else {
		return LoopPeriod::None;
	}
}

/* 组合所有周期标志位为位掩码
 * 一个周期可能同时是多个频率的执行点
 * 如周期0: 1000/500/250/20Hz全部执行(所有除法余数都为0)
 * 输出通过位或(|)组合，后续用位与(&)检查
 */
LoopPeriod MainLoop::makePeriodFlags() {
	/* 周期计数器0→999，达到1000时归零
	 * 防止整数溢出UB(虽然模运算在溢出时行为未必可靠)
	 */
	if (m_cycleCounter >= MAIN_LOOP_RATE) {
		m_cycleCounter = 0;
	}

	LoopPeriod lp = LoopPeriod::None;
	lp |= makePeriodFlag<LoopPeriod::Period1000hz>();
	lp |= makePeriodFlag<LoopPeriod::Period500hz>();
	lp |= makePeriodFlag<LoopPeriod::Period250hz>();
	lp |= makePeriodFlag<LoopPeriod::Period20hz>();

	m_cycleCounter++;

	return lp;
}
