#include "fluxeem_ros2_publisher/pixel_operations.hpp"

namespace pixel_operations
{
size_t stackTimeSurfaceRange(uint8_t *data, const Event *begin, const Event *end,
							 Timestamp start_timestamp, const RegionOfInterest &region_of_interest,
							 int width, int channel)
{
	if (data == nullptr || begin == end)
	{
		return 0;
	}

	const auto x_start = region_of_interest.x_start_;
	const auto x_end = region_of_interest.x_end_;
	const auto y_start = region_of_interest.y_start_;
	const auto y_end = region_of_interest.y_end_;
	size_t processed_count = 0;

	if (channel == 1)
	{
		for (const Event *event = begin; event != end; event++)
		{
			if (event->timestamp < start_timestamp || event->x < x_start || event->x >= x_end ||
				event->y < y_start || event->y >= y_end)
			{
				continue;
			}

			const int index = (event->y - y_start) * width + (event->x - x_start);
			data[index] = event->polarity ? POSITIVE_COLOR : NEGATIVE_COLOR;
			processed_count++;
		}
		return processed_count;
	}

	for (const Event *event = begin; event != end; event++)
	{
		if (event->timestamp < start_timestamp || event->x < x_start || event->x >= x_end || event->y < y_start ||
			event->y >= y_end)
		{
			continue;
		}

		uint8_t *pixel = data + ((event->y - y_start) * width + (event->x - x_start)) * 3;
		const int set_channel = event->polarity ? 2 : 0;
		const int clear_channel = event->polarity ? 0 : 2;
		pixel[set_channel] = POSITIVE_COLOR;
		pixel[clear_channel] = 0;
		processed_count++;
	}

	return processed_count;
}

size_t stackHistogramRange(uint8_t *data, const Event *begin, const Event *end,
						   Timestamp start_timestamp, const RegionOfInterest &region_of_interest,
						   int width, int channel)
{
	if (data == nullptr || begin == end)
	{
		return 0;
	}

	const auto x_start = region_of_interest.x_start_;
	const auto x_end = region_of_interest.x_end_;
	const auto y_start = region_of_interest.y_start_;
	const auto y_end = region_of_interest.y_end_;
	size_t processed_count = 0;

	if (channel == 1)
	{
		for (const Event *event = begin; event != end; event++)
		{
			if (event->timestamp < start_timestamp || event->x < x_start || event->x >= x_end ||
				event->y < y_start || event->y >= y_end)
			{
				continue;
			}

			uint8_t &pixel = data[(event->y - y_start) * width + (event->x - x_start)];
			pixel += pixel < 255 ? 1 : 0;
			processed_count++;
		}
		return processed_count;
	}

	for (const Event *event = begin; event != end; event++)
	{
		if (event->timestamp < start_timestamp || event->x < x_start || event->x >= x_end || event->y < y_start ||
			event->y >= y_end)
		{
			continue;
		}

		uint8_t *pixel = data + ((event->y - y_start) * width + (event->x - x_start)) * 3;
		uint8_t &channel_value = pixel[event->polarity ? 2 : 0];
		channel_value += channel_value < 255 ? 1 : 0;
		processed_count++;
	}

	return processed_count;
}
} // namespace pixel_operations
