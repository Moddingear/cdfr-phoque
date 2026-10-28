
#pragma once

#include <stdint.h>

#include <CANRobot.hpp>

namespace can_if
{
	void setup();

	void loop();

	bool send(RobotCAN::ID_extended target, const uint8_t* buffer, uint8_t length);

	bool send_fractionned(RobotCAN::ID_extended target, const uint8_t* buffer, uint8_t length);

	void send_error(RobotCAN::Error_packet::errors error);
	void send_hello();
	void send_feedback();
}