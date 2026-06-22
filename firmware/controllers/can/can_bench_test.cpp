/*
 * [can_bench_test.cpp - CAN测试工具]
 * 提供CAN总线上的引脚读写测试功能，用于硬件验证和调试。
 * 通过标准CAN消息读取/设置/清除ECU输出引脚状态。
 * 关键参数: CAN_BENCH_HEADER=0x66, 命令码(GET_COUNT/GET_SET/GET_CLEAR)
 */

#include "pch.h"
#include "can_bench_test.h"
#include "can_msg_tx.h"

#define CAN_BENCH_HEADER 0x66
#define CAN_BENCH_GET_COUNT 0
#define CAN_BENCH_GET_SET 1
#define CAN_BENCH_GET_CLEAR 2

#if EFI_CAN_SUPPORT

static void setPin(const CANRxFrame& frame, int value) {
	int index = frame.data8[1];
	if (index >= getBoardMetaOutputsCount()) {
		return;
	}

#if EFI_PROD_CODE
	Gpio pin = getBoardMetaOutputs()[index];
	palWritePad(getHwPort("can_write", pin), getHwPin("can_write", pin), value);
#endif // EFI_PROD_CODE
}

void processCanBenchTest(const CANRxFrame& frame) {
	if (CAN_EID(frame) != CAN_ECU_HW_META) {
		return;
	}
	if (frame.data8[0] != CAN_BENCH_HEADER) {
		return;
	}
	uint8_t command = frame.data8[1];
	if (command == CAN_BENCH_GET_COUNT) {
		CanTxMessage msg(CAN_ECU_HW_META + 1, 8);
		msg[0] = CAN_BENCH_HEADER;
		msg[1] = 0;
		msg[2] = getBoardMetaOutputsCount();

	} else if (command == CAN_BENCH_GET_SET) {
		setPin(frame, 1);
	} else if (command == CAN_BENCH_GET_CLEAR) {
		setPin(frame, 0);
	}
}

#endif // EFI_CAN_SUPPORT
