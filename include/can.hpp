
#pragma once

#include <stdint.h>

namespace CAN
{
	void init(uint16_t identifier, uint64_t frequency = 250000ULL);
}