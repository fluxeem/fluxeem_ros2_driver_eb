#include "fluxeem_ros2_publisher/modules.hpp"
#include "fluxeem_ros2_publisher/pixel_operations.hpp"

TimeSurfaceMemoryModule::TimeSurfaceMemoryModule(rclcpp::Node *node, const std::string &identifier,
												 const std::string &topic_name, size_t history_depth, double lifetime)
	: ImageMemoryModuleBase(node, identifier, topic_name, history_depth, lifetime, "time surface memory module")
{
}

int TimeSurfaceMemoryModule::initialValue() const
{
	return channel_ == 1 ? BG_COLOR : 0;
}

void TimeSurfaceMemoryModule::processEvent(const Event *event_ptr)
{
	processEventRange(event_ptr, event_ptr + 1);
}

void TimeSurfaceMemoryModule::processEventRange(const Event *begin, const Event *end)
{
	if (begin == end)
	{
		return;
	}

	std::lock_guard<std::shared_mutex> lock(this->shared_memory_mutex_);
	this->event_count_ += pixel_operations::stackTimeSurfaceRange(data_, begin, end, start_timestamp_,
																  region_of_interest_, width_, channel_);
}
