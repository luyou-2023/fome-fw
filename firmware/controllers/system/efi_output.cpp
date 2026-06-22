/*
 * @file efi_output.cpp
 *
 */

/*
 * [EFI输出] efi_output.cpp
 * EFI输出引脚抽象基类，定义输出引脚的公共接口。
 * 提供引脚状态追踪、PWM配置和逻辑电平控制。
 * 所有具体输出引脚类型（喷油器、点火、继电器等）均继承此类。
 */

#include "efi_output.h"
