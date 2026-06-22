// [中文标题] Hellen 144引脚 LED 引脚映射
Gpio getCommsLedPin() {
	return H144_LED3; // blue
}

Gpio getRunningLedPin() {
	return H144_LED2; // green
}

Gpio getWarningLedPin() {
	return H144_LED4; // yellow
}
