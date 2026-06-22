// [中文标题] Hellen 64引脚 LED 引脚映射
Gpio getCommsLedPin() {
	return H64_LED2_BLUE;
}

Gpio getWarningLedPin() {
	// this board has no warning led
	return Gpio::Unassigned;
}

Gpio getRunningLedPin() {
	// this board has no running led
	return Gpio::Unassigned;
}
