/**
 * [品牌触发解码器 - Mercedes/奔驰模式]
 * 配置奔驰M111车型的曲轴触发波形 (两段式模式)。
 * 6脉冲曲轴模式，每360度3个脉冲＋特殊同步间隙。
 * addEvent360调用定义齿的上升/下降沿角度位置。
 * setTriggerSynchronizationGap定义同步间隙比。
 */

#include "pch.h"

#include "trigger_mercedes.h"
#include "trigger_structure.h"

void setMercedesTwoSegment(TriggerWaveform* s) {
	s->initialize(FOUR_STROKE_CRANK_SENSOR, SyncEdge::Rise);

	s->addEvent360(180 - 10, true);
	s->addEvent360(180, false);

	s->addEvent360(227 - 10, true);
	s->addEvent360(227, false);

	s->addEvent360(360 - 10, true);
	s->addEvent360(360, false);

	s->setTriggerSynchronizationGap(1.35);
	s->setSecondTriggerSynchronizationGap(2.84);
	s->setThirdTriggerSynchronizationGap(0.26);
}
