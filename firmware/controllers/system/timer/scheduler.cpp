/**
 * @file	scheduler.h
 *
 * @date October 1, 2020
 */
/*
 * [调度器] scheduler.cpp
 * 发动机事件调度核心，将曲轴角度域事件转换为时间域调度。
 * action_s 封装回调函数及其参数，由调度器在精确时刻触发执行。
 * 依赖 event_queue 实现事件存储，配合硬件定时器产生精确中断。
 */
#include "pch.h"

#include "scheduler.h"

void action_s::execute() {
	efiAssertVoid(ObdCode::CUSTOM_ERR_ASSERT, m_callback != NULL, "callback==null1");
	m_callback(m_param);
}

schfunc_t action_s::getCallback() const {
	return m_callback;
}

void* action_s::getArgument() const {
	return m_param;
}
