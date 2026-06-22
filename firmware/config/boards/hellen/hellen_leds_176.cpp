// [中文标题] Hellen 176引脚 LED 引脚映射
Gpio getCommsLedPin() {
	return Gpio::H10; // blue
}

Gpio getRunningLedPin() {
	return Gpio::H9; // green
}

Gpio getWarningLedPin() {
	return Gpio::H11; // yellow
}
