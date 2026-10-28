#include <Arduino.h>
#include <main.hpp>
#include <algorithm>
#include <math.hpp>

#include <foc.hpp>
#include <can.hpp>
#include <USBSerial.h>

void jump_to_bootloader()
{
	//jump to 0x1FFF0000
	volatile uint32_t addr = 0x1FFF0000;
	HAL_RCC_DeInit();
	SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
	__disable_irq();
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	void (*SysMemBootJump)(void) = (void (*)(void)) (*((uint32_t *)(addr + 4)));
	__set_MSP(*(uint32_t *)addr);
	SysMemBootJump();
}

void setup()
{
	// use monitoring with serial
	Serial.begin(BAUD_RATE);

	#if defined(ARDUINO_PHOQUE)
	pinMode(BRAKE_RESISTOR, OUTPUT);
	digitalWrite(BRAKE_RESISTOR, LOW);
	#endif

	analogReadResolution(12);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(A_VBUS, INPUT_ANALOG);
	pinMode(A_TEMPERATURE, INPUT_ANALOG);
	pinMode(CAN_PWR, OUTPUT);
	digitalWrite(CAN_PWR, HIGH);

	while (FOC::get_temperature() > 80)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		Serial.printf("OVERTEMP: %d (%d), VBUS %f\r\n", FOC::get_temperature(), analogRead(A_TEMPERATURE), FOC::get_vbus());
	}
	
	for (size_t i = 0; i < 100; i++)
	{
		delay(100);
		digitalWrite(LED_BUILTIN, i & 1);
	}

	can_if::setup();

	can_if::send_hello();

	auto uid = get_chip_uid();
	Serial.printf("Driver ready. UID X %d Y %d Wafer %d Lot %7s\n", uid.x, uid.y, uid.wafer_num, uid.lot_number);
}

void loop() 
{
	bool led_status = (millis()%1000) > 500;
	digitalWriteFast(digitalPinToPinName(LED_BUILTIN), led_status);
	
	
	#if STANDALONE
	if (!FOC::IsSetup())
	{
		if (FOC::get_vbus() > 10)
		{
			vbus_ok++;
		}
		else
		{
			vbus_ok = 0;
		}


		//Serial.printf("Temperature: %d (%d), VBUS %f\r\n", FOC::get_temperature(), analogRead(A_TEMPERATURE), FOC::get_vbus());
		
		if (vbus_ok > 10)
		{
			Serial.printf("Voltage sufficient (%f), temp at %d, starting FOC.\n", FOC::get_vbus(), FOC::get_temperature());
			FOC::setup();
		}
	}
	#endif
	FOC::loop();
	can_if::loop();
}

struct RobotCAN::stm32_chip_uid get_chip_uid()
{
	struct RobotCAN::stm32_chip_uid uid_s;
	uint32_t *uid = reinterpret_cast<uint32_t*>(&uid_s);
	for (size_t i = 0; i < 3; i++)
	{
		uid[i] = ((uint32_t*)UID_BASE)[i];
	}
	return uid_s;
}