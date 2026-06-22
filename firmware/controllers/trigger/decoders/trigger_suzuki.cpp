/*
 * trigger_suzuki.cpp
 *
 * @date Oct 4, 2021
 * @author Andrey Belomutskiy, (c) 2012-2021
 */

/**
 * [品牌触发解码器 - Suzuki/铃木模式]
 * 配置铃木G13B车型的凸轮轴触发波形。
 * 4脉冲凸轮轴模式，含特殊长齿用于同步。
 * addEvent720调用定义齿的上升/下降沿角度位置。
 * setTriggerSynchronizationGap定义同步间隙比。
 */

#include "pch.h"

#include "trigger_suzuki.h"

void initializeSuzukiG13B(TriggerWaveform* s) {
	s->initialize(FOUR_STROKE_CAM_SENSOR, SyncEdge::RiseOnly);

	float w = 5;
	float specialTooth = 20;

	s->addEvent720(180 - w, true);
	s->addEvent720(180, false);

	s->addEvent720(2 * specialTooth + 180 - w, true);
	s->addEvent720(2 * specialTooth + 180, false);

	s->addEvent720(360 - w, true);
	s->addEvent720(360, false);

	s->addEvent720(540 - w, true);
	s->addEvent720(540, false);

	s->addEvent720(720 - w, true);
	s->addEvent720(720, false);

	s->setTriggerSynchronizationGap(0.22);
	s->setSecondTriggerSynchronizationGap(1);
}
