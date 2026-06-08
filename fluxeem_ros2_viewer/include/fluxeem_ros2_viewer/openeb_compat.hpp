#pragma once

#include <cstdint>

#include <metavision/sdk/base/utils/timestamp.h>

using Timestamp = Metavision::timestamp;

struct Event
{
	uint16_t x;
	uint16_t y;
	int16_t polarity;
	Timestamp timestamp;
};
