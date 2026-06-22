/*
 * [can_filter.cpp - CAN帧过滤器]
 * 实现CAN消息ID过滤机制，支持按总线索引和ID匹配。
 * 最大支持48个过滤器，使用CCM_OPTIONAL优化内存布局。
 * 关键参数: maxFilterCount=48, CanBusIndex总线枚举
 */
#include "pch.h"
#include "can_filter.h"
#include "can_hw.h"

static constexpr size_t maxFilterCount = 48;

static size_t filterCount = 0;
static CCM_OPTIONAL CanFilter filters[maxFilterCount];

CanFilter* getFilterForId(CanBusIndex busIndex, int Id) {
	for (size_t i = 0; i < filterCount; i++) {
		auto& filter = filters[i];

		if (filter.accept(Id)) {
			if (filter.Bus == CanBusIndex::Any || filter.Bus == busIndex) {
				return &filter;
			}
		}
	}

	return nullptr;
}

void resetLuaCanRx() {
	// Clear all lua filters - reloading the script will reinit them
	filterCount = 0;
}

void addLuaCanRxFilter(int32_t eid, uint32_t mask, CanBusIndex bus, int callback) {
	if (filterCount >= maxFilterCount) {
		firmwareError("Too many Lua CAN RX filters");
	}

	efiPrintf(
			"Added Lua CAN RX filter id 0x%x mask 0x%x with%s custom function",
			(unsigned int)eid,
			(unsigned int)mask,
			(callback == -1 ? "out" : ""));

	filters[filterCount].Id = eid;
	filters[filterCount].Mask = mask;
	filters[filterCount].Bus = bus;
	filters[filterCount].Callback = callback;

	filterCount++;
}
