#if PHOQUE
#include <variant_PHOQUE.h>
#endif
#include <Arduino.h>
#include <algorithm>
#include <math.hpp>

#include <foc.hpp>

#if SIMPLECANLIB
#include <SimpleCAN.h>
#elif HAL_CAN
#include <can.hpp>
#endif
#include <CANRobot.hpp>
#include <map>
#include <USBSerial.h>

#if PHOQUE
HardwareSerial Serial1(A_USART_RX, A_USART_TX);
#endif

int potentiometer_value = 0;
int last_message_timestamp = 0;

std::map<int16_t, uint64_t> discriminators;


void (*FOCFunctiontoRun)(void) = nullptr;

#if SIMPLECANLIB
SimpleCan* CanInstance;
FilterDefinition filter[] ={
{ //accept only us
	.FilterIndex = 0,
	.FilterType = CAN_FILTER_MASK,
	.FilterConfig = CAN_FILTER_TO_RXFIFO0,
	.IdType = CAN_STDID
},
#if 0
{ //reject all stdid
	.FilterIndex = 1,
	.FilterType = CAN_FILTER_MASK,
	.FilterConfig = CAN_FILTER_REJECT,
	.IdType = CAN_STDID,
	.FilterID1 = 0,
	.FilterID2 = 0,
},
{ //reject all extid
	.FilterIndex = 2,
	.FilterType = CAN_FILTER_MASK,
	.FilterConfig = CAN_FILTER_REJECT,
	.IdType = CAN_EXTID,
	.FilterID1 = 0,
	.FilterID2 = 0,
}
#endif
};
#else

#endif

uint32_t can_identifier = 0;

void SendError(RobotCAN::Error_packet::errors error)
{
	RobotCAN::Error_packet message;
	message.error = error;
	#if SIMPLECANLIB
	CanInstance->SendMessage(reinterpret_cast<uint8_t*>(&message), sizeof(message), RobotCAN::master_address_high_priority);
	#endif
}

void SendHello()
{
	last_message_timestamp = millis();
	RobotCAN::hello_packet message;
	message.discriminator = can_identifier;
	#if SIMPLECANLIB
	CanInstance->SendMessage(reinterpret_cast<uint8_t*>(&message), sizeof(message), RobotCAN::master_address_low_priority);
	#endif
}

#if SIMPLECANLIB
void OnReceive(const SimpleCanRxHeader rxHeader, const uint8_t *rxData, void* userData)
{
	if (rxHeader.DataLength < 1)
	{
		Serial.printf("Message too short, ignored (ID %d, length %d)\n", rxHeader.Identifier, rxHeader.DataLength);
		return;
	}
	
	last_message_timestamp = millis();
	auto type = *reinterpret_cast<const RobotCAN::PacketType*>(&rxData[0]);
	switch (type)
	{
	case RobotCAN::PacketType::VERSION_CHECK :
		if (rxHeader.DataLength == sizeof(RobotCAN::Version_check))
		{
			auto message = reinterpret_cast<const RobotCAN::Version_check*>(rxData);
			if (message->version != RobotCAN::protocol_version)
			{
				SendError(RobotCAN::Error_packet::errors::VERSION_MISMATCH);
				Serial.printf("Version mismatch ! We have %d, they have %d\n", RobotCAN::protocol_version, message->version);
			}
			else
			{
				Serial.printf("Version matches\n");
			}
			
		}
		break;
	case RobotCAN::PacketType::INIT_FOC :
		if (rxHeader.Identifier != can_identifier)
		{
			break;
		}
		Serial.printf("Starting FOC...\n");
		FOC::setup();
		if (!FOC::IsSetup())
		{
			SendError(RobotCAN::Error_packet::errors::NOT_INITIATED);
		}
		break;
	case RobotCAN::PacketType::ENABLE_FOC :
		if (rxHeader.Identifier != can_identifier)
		{
			break;
		}
		Serial.printf("Enabling motor...\n");
		FOC::start();
		break;
	case RobotCAN::PacketType::STOP_ALL :
		if (rxHeader.Identifier != can_identifier)
		{
			break;
		}
		Serial.printf("Disabling motor...\n");
		FOC::set_target(0);
		FOC::stop();
		break;

	case RobotCAN::PacketType::MOTOR_TARGET :
		if (rxHeader.Identifier != can_identifier)
		{
			break;
		}
		if (rxHeader.DataLength == sizeof(RobotCAN::Motor_target_packet))
		{
			auto message = reinterpret_cast<const RobotCAN::Motor_target_packet*>(rxData);
			FOC::set_target(message->target);
		}
		break;

	case RobotCAN::PacketType::MOTOR_DRIVE_MODE :
		if (rxHeader.Identifier != can_identifier)
		{
			break;
		}
		if (rxHeader.DataLength == sizeof(RobotCAN::Motor_mode_packet))
		{
			auto message = reinterpret_cast<const RobotCAN::Motor_mode_packet*>(rxData);
			MotionControlType type = MotionControlType::torque;
			switch (message->mode)
			{
			case RobotCAN::Motor_mode_packet::mode_select::angle :
				type = MotionControlType::angle;
				break;
			
			case RobotCAN::Motor_mode_packet::mode_select::velocity :
				type = MotionControlType::velocity;
				break;
			
			case RobotCAN::Motor_mode_packet::mode_select::torque :
				type = MotionControlType::torque;
				break;
			
			default:
				break;
			}
			FOC::set_motion_mode(type);
		}
		break;
	
	case RobotCAN::PacketType::PID_CHANGE_SETTING :
		if (rxHeader.Identifier != can_identifier)
		{
			break;
		}
		if (rxHeader.DataLength == sizeof(RobotCAN::PIDTunePacket))
		{
			auto message = reinterpret_cast<const RobotCAN::PIDTunePacket*>(rxData);
			FOC::tune_settings(message);
		}
		break;

	case RobotCAN::PacketType::HELLO :
		if (rxHeader.DataLength == sizeof(RobotCAN::hello_packet))
		{
			auto message = reinterpret_cast<const RobotCAN::hello_packet*>(rxData);
			int16_t discriminator = message->discriminator;
			auto key = discriminators.find(discriminator);
			if (key == discriminators.end())
			{
				discriminators[discriminator] = millis();
				printf("Enumeration : Got a motor with discriminator %d\n", discriminator);
			}
			else
			{
				key->second = millis();
			}
			
		}
		break;
	case RobotCAN::PacketType::MOTOR_FEEDBACK :
		break;

	default:
		Serial.printf("Unhandled packet identifier %#x from %#x\n", type, rxHeader.Identifier);
		break;
	}
}

bool FilterFunc(int CanID)
{
	return CanID == can_identifier;
}
#endif

void setup()
{
	// use monitoring with serial
	Serial.begin(BAUD_RATE);
	#if !PHOQUE
	pinMode(A_BUTTON, INPUT);
	#endif
	pinMode(LED_BUILTIN, OUTPUT);

	while (HSE_VALUE != 8000000UL)
	{
		//go set HSE_VALUE to 8000000UL in .platformio/packages/framework-arduinoststm32/variants/STM32G4xx/G431C(6-8-B)U_G441CBU/variant_B_G431B_ESC1.h
		Serial.println("Wrong HSE value!");
	}

	analogReadResolution(12);
	potentiometer_value = analogRead(A_POTENTIOMETER);
	can_identifier = potentiometer_value >>1;
	if (can_identifier == RobotCAN::master_address_low_priority)
	{
		can_identifier = RobotCAN::master_address_low_priority-1;
	}
	if (can_identifier == RobotCAN::master_address_high_priority)
	{
		can_identifier = RobotCAN::master_address_high_priority+1;
	}
	#if SIMPLECANLIB
	filter[0].FilterID1 = can_identifier;
	filter[0].FilterID2 = 0x7ff;
	#endif
	



	bool end_of_line = potentiometer_value < 100 || potentiometer_value > (4095-100);

	Serial.printf("Potentiometer value is %d, end of the line is %d, we are ID %d\n", potentiometer_value, end_of_line, can_identifier);

	//FOC::setup();
	#if !PHOQUE
	pinMode(A_CAN_TERM, OUTPUT);
	digitalWrite(A_CAN_TERM, end_of_line);
	#endif

	#if SIMPLECANLIB
	CanInstance = CreateCanLib(A_CAN_TX, A_CAN_RX);
	CanInstance->Init(Kbit250);
	CanInstance->Start();
	CanInstance->SetBusTermination(end_of_line);
	CanInstance->ActivateNotification(0, OnReceive, nullptr);
	for (size_t i = 0; i < sizeof(filter)/sizeof(filter[0]); i++)
	{
		auto status = CanInstance->ConfigFilter(&filter[i]);
		Serial.printf("Filter %d status %d\n", i, status);
	}
	CanInstance->ConfigGlobalFilter();
	#else
	
	#endif

	SendHello();

	Serial.println("Driver ready.");
}

void loop() 
{
	static uint64_t last_feedback = millis();
	bool led_status = (millis()%1000) > 500;
	digitalWrite(LED_BUILTIN, led_status);
	
	#if !PHOQUE
	if (!digitalRead(A_BUTTON))
	{
		Serial.printf("Resetting...\n");
		NVIC_SystemReset();
	}
	#endif
	#if SIMPLECANLIB
	CanInstance->Loop();
	#endif
	#if STANDALONE
	if (!FOC::IsSetup() && FOC::get_vbus() > 10)
	{
		Serial.printf("Voltage sufficient (%f), starting FOC.\n", FOC::get_vbus());
		FOC::setup();
	}
	#endif
	FOC::loop();
	if (FOC::IsSetup() && last_feedback + 100 < millis())
	{
		last_feedback = millis();
		RobotCAN::Motor_feedback message;
		message.angle = FOC::get_angle();
		message.temperature = FOC::get_temperature()*2;
		message.current = FOC::get_current()/10;
		#if SIMPLECANLIB
		CanInstance->SendMessage(reinterpret_cast<uint8_t*>(&message), sizeof(message), RobotCAN::master_address_low_priority);
		#endif
	}
	if (last_message_timestamp + 5000 < millis())
	{
		SendHello();
	}
	
}