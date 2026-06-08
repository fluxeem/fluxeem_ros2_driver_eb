#pragma once

#include "event_topic_module.hpp"
#include <cstddef>
#include <cstdint>

namespace pixel_operations
{
size_t stackTimeSurfaceRange(uint8_t *data, const Event *begin, const Event *end,
							 Timestamp start_timestamp, const RegionOfInterest &region_of_interest,
							 int width, int channel);

size_t stackHistogramRange(uint8_t *data, const Event *begin, const Event *end,
						   Timestamp start_timestamp, const RegionOfInterest &region_of_interest,
						   int width, int channel);
} // namespace pixel_operations
