#pragma once

#include <common/base_classes/FOCMotor.h>
#include <CANRobot.hpp>

namespace FOC
{
	void setup();
	bool IsSetup();
	void loop();
	void start();
	void stop();
	void set_target(float target);
	void set_motion_mode(MotionControlType type);
	float get_angle();
	float get_temperature();
	float get_current();

	void tune_settings(const RobotCAN::PIDTunePacket *message);
};