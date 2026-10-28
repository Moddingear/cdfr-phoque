#include <can.hpp>

#include <Arduino.h>
#include <foc.hpp>
#include <CANRobot.hpp>
#include <main.hpp>
#if SIMPLECANLIB
#include <ComplexCAN_STM32.h>
#endif

uint16_t potentiometer_value = 0;
uint8_t discriminator = 0, next_discriminator = 0;
uint64_t last_message_timestamp = 0;
int vbus_ok = 0;

const uint8_t discriminator_shift = 4;


void (*FOCFunctiontoRun)(void) = nullptr;

#if SIMPLECANLIB
ComplexCan_STM32 CanInstance(digitalPinToPinName(CAN_TX), digitalPinToPinName(CAN_RX), FDCAN1, GPIO_AF9_FDCAN1, GPIO_AF9_FDCAN1);
#if !DEMO
FilterDefinition filter[] ={
{ //accept only us
	.FilterIndex = 0,
	.FilterType = CAN_FILTER_MASK,
	.FilterConfig = CAN_FILTER_TO_RXFIFO0,
	.ExtendedID = false,
	.FilterID2 = RobotCAN::motor_mask_std,
},
{ //accept only us
	.FilterIndex = 0,
	.FilterType = CAN_FILTER_MASK,
	.FilterConfig = CAN_FILTER_TO_RXFIFO0,
	.ExtendedID = true,
	.FilterID2 = RobotCAN::motor_mask_extd,
},
};
#endif
#endif

#if SIMPLECANLIB
void OnReceive(const ComplexCanRxHeader &rxHeader, const uint8_t *rxData, void* userData)
{
	#if !DEMO
	if (rxHeader.DataLength < 1-rxHeader.ExtendedID)
	{
		Serial.printf("Message too short, ignored (ID %d, length %d)\n", rxHeader.Identifier, rxHeader.DataLength);
		return;
	}
	
	last_message_timestamp = millis();
	RobotCAN::PacketType type;
	bool targetting_this;
	const uint8_t *data_ptr;
	uint8_t data_length;
	if (rxHeader.ExtendedID)
	{
		auto header = RobotCAN::ID_extended(rxHeader.Identifier);
		targetting_this = header.motor_index == discriminator;
		type = static_cast<RobotCAN::PacketType>(header.packet_type);
		data_ptr = rxData;
		data_length = rxHeader.DataLength;
	}
	else
	{
		auto header = RobotCAN::ID_standard(rxHeader.Identifier);
		targetting_this = header.motor_index == discriminator;
		type = static_cast<RobotCAN::PacketType>(rxData[0]);
		data_ptr = &rxData[1];
		data_length = rxHeader.DataLength-1;
	}
	
	
	switch (type)
	{
	case RobotCAN::PacketType::VERSION_CHECK :
		if (data_length == sizeof(RobotCAN::Version_check))
		{
			auto message = reinterpret_cast<const RobotCAN::Version_check*>(data_ptr);
			if (message->version != RobotCAN::protocol_version)
			{
				can_if::send_error(RobotCAN::Error_packet::errors::VERSION_MISMATCH);
				Serial.printf("Version mismatch ! We have %d, they have %d\n", RobotCAN::protocol_version, message->version);
			}
			else
			{
				Serial.printf("Version matches\n");
			}
			
		}
		break;
	case RobotCAN::PacketType::INIT_FOC :
		if (!targetting_this)
		{
			break;
		}
		Serial.printf("Starting FOC...\n");
		//CanInstance->DeactivateNotification();
		FOC::setup();
		if (!FOC::IsSetup())
		{
			can_if::send_error(RobotCAN::Error_packet::errors::NOT_INITIATED);
		}
		//CanInstance->ActivateNotification(0, OnReceive, nullptr);
		break;
	case RobotCAN::PacketType::ENABLE_FOC :
		if (!targetting_this)
		{
			break;
		}
		Serial.printf("Enabling motor...\n");
		FOC::start();
		break;
	case RobotCAN::PacketType::STOP_ALL :
		Serial.printf("Disabling motor...\n");
		FOC::set_target(0);
		FOC::stop();
		break;

	case RobotCAN::PacketType::MOTOR_TARGET :
		if (!targetting_this)
		{
			break;
		}
		if (data_length == sizeof(RobotCAN::Motor_target_packet))
		{
			auto message = reinterpret_cast<const RobotCAN::Motor_target_packet*>(data_ptr);
			FOC::set_target(message->target);
		}
		break;

	case RobotCAN::PacketType::MOTOR_DRIVE_MODE :
		if (!targetting_this)
		{
			break;
		}
		if (data_length == sizeof(RobotCAN::Motor_mode_packet))
		{
			auto message = reinterpret_cast<const RobotCAN::Motor_mode_packet*>(data_ptr);
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
	
	case RobotCAN::PacketType::PID_SETTING :
		{
			if (!targetting_this)
			{
				break;
			}
			bool has_new_value = data_length == sizeof(RobotCAN::PIDTunePacket);
			bool value_request = data_length == sizeof(RobotCAN::PIDTunePacket) - sizeof(RobotCAN::PIDTunePacket::new_value);
			if (has_new_value || value_request)
			{
				auto message = reinterpret_cast<const RobotCAN::PIDTunePacket*>(data_ptr);
				Serial.printf("Got PID packet for %d/%d, request ?%d\n", message->parameter, message->loop, value_request);
				FOC::tune_settings(message, value_request);
			}
			else
			{
				Serial.printf("PID packet invalid length %d\n", data_length);
			}
		}
		break;

	case RobotCAN::PacketType::HELLO :
		break;
	case RobotCAN::PacketType::MOTOR_FEEDBACK :
		if (targetting_this && rxHeader.RemoteFrame)
		{
			can_if::send_feedback();
		}
		break;
	
	case RobotCAN::PacketType::ENTER_BOOTLOADER :
		if (targetting_this)
		{
			Serial.printf("Jumping to bootloader!\n");
			jump_to_bootloader();
		}
		break;
	case RobotCAN::PacketType::IDENTIFY :
		if (targetting_this)
		{
			auto uid = get_chip_uid();
			RobotCAN::stm32_identification_fdpacket packet;
			packet.subfamily[0] = 'G';
			packet.subfamily[1] = '4';
			packet.uid=uid;
			if (rxHeader.FDCANFrame)
			{
				Serial.printf("Sending UID as single FDCAN frame\n");
				can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::IDENTIFY, discriminator, 0, true), reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
			}
			else
			{
				Serial.printf("Sending UID as fractionned CAN frame\n");
				can_if::send_fractionned(RobotCAN::ID_extended(0, RobotCAN::PacketType::IDENTIFY, discriminator, 0, true), reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
			}
		}
		break;
	default:
		Serial.printf("Unhandled packet identifier %#x from %#x\n", type, rxHeader.Identifier);
		break;
	}
	#else 
	if (rxHeader.DataLength == sizeof(float))
	{
		bool greater = rxHeader.Identifier > can_identifier;
		const float magnification = 2.f;
		float target = *reinterpret_cast<const float*>(rxData) * (greater ? magnification : 1/magnification);
		FOC::set_target(target);
	}
	#endif
}
#endif


void can_if::setup()
{
	potentiometer_value = FOC::get_pot();
	discriminator = potentiometer_value >> discriminator_shift; //discriminator is 8bit, ADC is 12 bit
	next_discriminator = discriminator;
	#if SIMPLECANLIB && !DEMO
	filter[0].FilterID1 = RobotCAN::ID_standard(discriminator, 0, false, false);
	filter[1].FilterID1 = RobotCAN::ID_extended(0, static_cast<RobotCAN::PacketType>(0), discriminator, 0, 0, 0);
	#endif


	bool end_of_line = potentiometer_value < 100 || potentiometer_value > (4095-100);

	Serial.printf("Potentiometer value is %d, end of the line is %d, we are ID %d\n", potentiometer_value, end_of_line, discriminator);

	//FOC::setup();

	#if SIMPLECANLIB
	CanInstance.Init(1000000, 5000000);
	CanInstance.ActivateNotification(0, OnReceive, nullptr);
	for (size_t i = 0; i < sizeof(filter)/sizeof(filter[0]); i++)
	{
		auto status = CanInstance.ConfigFilter(&filter[i]);
		Serial.printf("Filter %d status %d\n", i, status);
	}
	CanInstance.ConfigGlobalFilter();
	CanInstance.Start();
	#endif
}

void can_if::loop()
{
	static uint64_t last_feedback = millis();
	#if SIMPLECANLIB
	CanInstance.Loop();
	#endif
	#if 1
	if (FOC::IsSetup() && last_feedback + 10 <= millis())
	{
		last_feedback = millis();
		#if !DEMO
		send_feedback();
		#else
		float angle = FOC::get_angle();
		CanInstance->SendMessage(reinterpret_cast<uint8_t*>(&angle), sizeof(angle), can_identifier);
		#endif
	}
	#endif
	#if 1
	if (last_message_timestamp + 5000 < millis())
	{
		send_hello();
	}
	#endif
	#if 0
	uint16_t potval = FOC::get_pot() >> discriminator_shift;
	if (potval != next_discriminator)
	{
		next_discriminator = potval;
		Serial.printf("Discriminator will change to %d\n", potval);
	}
	#endif
}

bool can_if::send(RobotCAN::ID_extended target, const uint8_t* buffer, uint8_t length)
{
	#if SIMPLECANLIB
	target.motor_index = discriminator;
	return CanInstance.SendMessage(buffer, length, target, true);
	#endif
}

bool can_if::send_fractionned(RobotCAN::ID_extended target, const uint8_t* buffer, uint8_t length)
{
	RobotCAN::PacketType type = target.packet_type;
	target.packet_type = RobotCAN::PacketType::FRACTION;
	uint8_t fraction_data[8];
	uint8_t sent, left, to_send;
	uint8_t num_packets = (length+2+7)/8; //round up
	Serial.printf("Sending fractionned packet (%d bytes, over %d packets)\n", length, num_packets);
	for (target.packet_index = 0; target.packet_index < num_packets; target.packet_index++)
	{
		if (target.packet_index == 0)
		{
			//send length + type + 6
			fraction_data[0] = num_packets;
			fraction_data[1] = static_cast<uint8_t>(type);
			to_send = 8;
			memcpy(&fraction_data[2], buffer, 6);
		}
		else
		{
			sent = target.packet_index*8-2;
			left = length - sent;
			to_send = std::min<uint8_t>(left, 8);
			memcpy(&fraction_data[0], &buffer[sent], to_send);
		}
		Serial.printf("Sending fraction packet %d (%d bytes)\n", target.packet_index, to_send);
		send(target, fraction_data, to_send);
	}
	return true;
}

#if !DEMO
void can_if::send_error(RobotCAN::Error_packet::errors error)
{
	RobotCAN::Error_packet message;
	message.error = error;
	can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::ERROR_PACKET, discriminator, 0, true, 1), reinterpret_cast<uint8_t*>(&message), sizeof(message));
}

void can_if::send_hello()
{
	last_message_timestamp = millis();
	Serial.printf("Sending hello from %d\n", discriminator);
	can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::HELLO, discriminator, 0, true), nullptr, 0);
}

void can_if::send_feedback()
{
	RobotCAN::Motor_feedback message;
	message.angle = FOC::get_angle();
	message.temperature = FOC::get_temperature()*2;
	message.current = FOC::get_current()/10;
	//Serial.printf("Temperature: %d, VBUS: %f\r\n", FOC::get_temperature(), FOC::get_vbus());
	can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::MOTOR_FEEDBACK, discriminator, 0, true), reinterpret_cast<uint8_t*>(&message), sizeof(message));
}
#endif
