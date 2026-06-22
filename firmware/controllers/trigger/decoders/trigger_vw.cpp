/*
 * @file trigger_vw.cpp
 *
 * @date Aug 25, 2018
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

/**
 * [品牌触发解码器 - VW/大众模式]
 * 配置大众/Škoda车型的曲轴触发波形。
 * 包括: Škoda Favorit (3齿模式)、VW通用 (60-2 缺齿模式)。
 * addEvent360/addEvent720调用定义齿的上升/下降沿角度位置。
 * setTriggerSynchronizationGap定义同步间隙比。
 */

#include "pch.h"
#include "trigger_vw.h"
#include "trigger_universal.h"

void setSkodaFavorit(TriggerWaveform* s) {
	s->initialize(FOUR_STROKE_CRANK_SENSOR, SyncEdge::Rise);

	s->addEvent360(46, true, TriggerWheel::T_PRIMARY);
	s->addEvent360(177, false, TriggerWheel::T_PRIMARY);

	s->addEvent360(180, true, TriggerWheel::T_PRIMARY);
	s->addEvent360(183, false, TriggerWheel::T_PRIMARY);

	s->addEvent360(226, true, TriggerWheel::T_PRIMARY);
	s->addEvent360(360, false, TriggerWheel::T_PRIMARY);

	s->tdcPosition = 180 - 46;
	s->setTriggerSynchronizationGap(3.91);
}

void setVwConfiguration(TriggerWaveform* s) {
	s->initialize(FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);

	int totalTeethCount = 60;
	int skippedCount = 2;

	float engineCycle = FOUR_STROKE_ENGINE_CYCLE;
	float toothWidth = 0.5;

	addSkippedToothTriggerEvents(TriggerWheel::T_PRIMARY, s, 60, 2, toothWidth, 0, engineCycle, NO_LEFT_FILTER, 690);

	float angleDown = engineCycle / totalTeethCount * (totalTeethCount - skippedCount - 1 + (1 - toothWidth));
	s->addEventClamped(0 + angleDown + 12, true, TriggerWheel::T_PRIMARY, NO_LEFT_FILTER, NO_RIGHT_FILTER);
	s->addEventClamped(0 + engineCycle, false, TriggerWheel::T_PRIMARY, NO_LEFT_FILTER, NO_RIGHT_FILTER);

	s->setTriggerSynchronizationGap2(1.6, 4);
	s->setSecondTriggerSynchronizationGap(1); // this gap is not required to synch on perfect signal but is needed to
											  // handle to reject cranking transition noise
	s->setThirdTriggerSynchronizationGap(1);
}
