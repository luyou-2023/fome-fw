/*
 * @file shutdown_controller.cpp
 *
 */

/* [关机控制器] 控制引擎停机流程 */

#include "pch.h"

void doScheduleStopEngine() {
	efiPrintf("Starting doScheduleStopEngine");
	getLimpManager()->shutdownController.stopEngine();
	// todo: initiate stepper motor parking
}
