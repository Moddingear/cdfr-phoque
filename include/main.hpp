#pragma once

#include <CANRobot.hpp>

void jump_to_bootloader();

struct RobotCAN::stm32_chip_uid get_chip_uid();