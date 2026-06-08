#include "fluxeem_ros2_publisher/modules.hpp"
#include "fluxeem_ros2_publisher/pixel_operations.hpp"

HistogramMemoryModule::HistogramMemoryModule(rclcpp::Node *node, const std::string &identifier,
											 const std::string &topic_name, size_t history_depth, double lifetime)
	: ImageMemoryModuleBase(node, identifier, topic_name, history_depth, lifetime, "histogram memory module")
{
}

int HistogramMemoryModule::initialValue() const
{
	return 0;
}

void HistogramMemoryModule::processEvent(const Event *event_ptr)
{
	processEventRange(event_ptr, event_ptr + 1);
}

void HistogramMemoryModule::processEventRange(const Event *begin, const Event *end)
{
	if (begin == end)
	{
		return;
	}

	std::lock_guard<std::shared_mutex> lock(this->shared_memory_mutex_);
	this->event_count_ += pixel_operations::stackHistogramRange(data_, begin, end, start_timestamp_,
																region_of_interest_, width_, channel_);
}
