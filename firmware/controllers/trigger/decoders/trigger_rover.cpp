/*
 * @file trigger_rover.cpp
 *
 * @date Dec 27, 2015
 * @author PhilTheGeek
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

/**
 * [品牌触发解码器 - Rover/罗孚模式]
 * 配置罗孚K系列发动机的曲轴触发波形。
 * 每90度1个齿的4齿曲轴模式，含特殊同步间隙。
 * addEvent720调用定义齿的上升/下降沿角度位置。
 * setTriggerSynchronizationGap定义同步间隙比。
 */

#include "pch.h"

#include "trigger_rover.h"

/**
 * https://en.wikipedia.org/wiki/Rover_K-series_engine
 */
void initializeRoverK(TriggerWaveform* s) {
	s->initialize(FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);

	float tooth = 20;

	s->setTriggerSynchronizationGap(2.0);
	// wow that's odd
	s->setSecondTriggerSynchronizationGap2(0.0001, 100000);
	s->setThirdTriggerSynchronizationGap(2);

	float base = 0;

	for (int i = 0; i < 2; i++) {
		s->addEvent720(base + tooth / 2, true);
		s->addEvent720(base + tooth, false);
		base += tooth;
	}
	base += tooth;
	for (int i = 0; i < 2; i++) {
		s->addEvent720(base + tooth / 2, true);
		s->addEvent720(base + tooth, false);
		base += tooth;
	}
	base += tooth;
	for (int i = 0; i < 14; i++) {
		s->addEvent720(base + tooth / 2, true);
		s->addEvent720(base + tooth, false);
		base += tooth;
	}
	base += tooth;
	for (int i = 0; i < 3; i++) {
		s->addEvent720(base + tooth / 2, true);
		s->addEvent720(base + tooth, false);
		base += tooth;
	}
	base += tooth;
	for (int i = 0; i < 11; i++) {
		s->addEvent720(base + tooth / 2, true);
		s->addEvent720(base + tooth, false);
		base += tooth;
	}
}
