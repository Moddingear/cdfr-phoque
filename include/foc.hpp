#pragma once

#include <common/base_classes/FOCMotor.h>
#include <CANRobot.hpp>

namespace FOC
{
	void setup();
	bool IsSetup();
	void RealTimeLoop();
	void loop();
	void start();
	void stop();
	void set_target(float target);
	void set_motion_mode(MotionControlType type);
	float get_angle();
	int get_temperature();
	float get_current();
	float get_vbus();
	uint16_t get_pot();

	void tune_settings(const RobotCAN::PIDTunePacket *message, bool value_request);
};