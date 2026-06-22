/**
 * @file	signature.cpp
 * @brief A special file which is recompiled every time the .ini file changes.
 *
 * This is a minimalistic fast-compiling cpp-file. Any additional massive includes are not welcomed.
 *
 * @date Jul 2, 2020
 * @author andreika (c) 2020
 */

// [中文标题] signature.cpp - 固件签名
// 每次ini文件变更时重新编译，返回TunerStudio通信签名标识符。

#include "pch.h"

#pragma message("TS_SIGNATURE: " TS_SIGNATURE)

const char* getTsSignature() {
	return TS_SIGNATURE;
}
