#include "fluxeem_ros2_publisher/modules.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <sys/stat.h>

static constexpr size_t DATA_SIZE_PER_UINT = 6; // An assumed number of events per microsecond

EventMemoryModule::EventMemoryModule(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name,
									 size_t history_depth, double lifetime)
	: EventTopicModule(node, identifier, topic_name, history_depth, lifetime), events_(nullptr),
	  spatial_sample_rate_(1.0), spatial_sample_rate_reciprocal_(1.0), spatial_sample_rate_is_one_(true),
	  interval_event_count_(0), accumulation_start_timestamp_(0), replay_window_accumulation_start_timestamp_(0),
	  replay_window_temporal_sample_rate_(1), replay_window_state_captured_(false), reserved_data_multiplier_(1),
	  event_capacity_(0), mapped_data_size_(0)
{
}

EventMemoryModule::~EventMemoryModule()
{
	if (adjust_timer_)
	{
		adjust_timer_->cancel();
	}

	std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);

	if (events_ != nullptr)
	{
		if (munmap((void *)events_, mapped_data_size_) == -1)
		{
			RCLCPP_WARN(node_->get_logger(), "Unmap the data pointer has failed.");
		}
	}

	RCLCPP_INFO(node_->get_logger(), "Cleaning the shared memory of event memory module");
}

void EventMemoryModule::setupModule(RegionOfInterest region_of_interest, double spatial_sample_rate,
									int temporal_sample_rate_min, size_t target_num_events_per_sec, int timespan)
{
	if (!std::isfinite(spatial_sample_rate) || spatial_sample_rate < 1.0)
	{
		throw std::invalid_argument("event memory module spatial sample rate must be finite and at least 1");
	}
	if (temporal_sample_rate_min <= 0)
	{
		throw std::invalid_argument("event memory module temporal sample rate minimum must be greater than 0");
	}

	region_of_interest_ = region_of_interest;
	spatial_sample_rate_ = spatial_sample_rate;
	spatial_sample_rate_reciprocal_ = 1.0 / spatial_sample_rate_;
	spatial_sample_rate_is_one_ = spatial_sample_rate_ == 1.0;
	temporal_sample_rate_min_ = temporal_sample_rate_min;
	temporal_sample_rate_ = temporal_sample_rate_min;
	target_num_events_per_sec_ = target_num_events_per_sec;
	if (target_num_events_per_sec_ != 0 && !adjust_timer_)
	{
		adjust_timer_ =
			node_->create_wall_timer(std::chrono::milliseconds(33), std::bind(&EventMemoryModule::adjust, this));
	}
	configureTimeWindow(timespan, "event memory module");
	interval_event_count_ = 0;
	replay_window_accumulation_start_timestamp_ = accumulation_start_timestamp_;
	replay_window_temporal_sample_rate_ = temporal_sample_rate_;
	replay_window_state_captured_ = false;

	std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);

	auto data_size = sizeof(Event) * DATA_SIZE_PER_UINT * static_cast<size_t>(timespan);
	events_ = static_cast<Event *>(createSharedMemory(data_size, -1));
	event_capacity_ = DATA_SIZE_PER_UINT * static_cast<size_t>(timespan);
	mapped_data_size_ = data_size;
	reserved_data_multiplier_ = 1;
	initialized_ = events_ != nullptr;
}

void EventMemoryModule::reset()
{
	std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);

	event_count_ = 0;
	interval_event_count_ = 0;
	resetTimeWindow();
	accumulation_start_timestamp_ = 0;
	temporal_sample_rate_ = temporal_sample_rate_min_;
	event_count_history_.clear();
	replay_window_accumulation_start_timestamp_ = accumulation_start_timestamp_;
	replay_window_temporal_sample_rate_ = temporal_sample_rate_;
	replay_window_state_captured_ = false;
}

void EventMemoryModule::processEvent(const Event *event)
{
	processEventRange(event, event + 1);
}

void EventMemoryModule::processEventRange(const Event *begin, const Event *end)
{
	if (begin == end)
	{
		return;
	}

	std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);
	captureReplayWindowState();

	interval_event_count_ += static_cast<size_t>(end - begin);
	const auto x_start = region_of_interest_.x_start_;
	const auto x_end = region_of_interest_.x_end_;
	const auto y_start = region_of_interest_.y_start_;
	const auto y_end = region_of_interest_.y_end_;
	const bool spatial_rate_is_one = spatial_sample_rate_is_one_;
	const double spatial_rate_reciprocal = spatial_sample_rate_reciprocal_;
	const int temporal_sample_rate = temporal_sample_rate_;

	for (const Event *event = begin; event != end; event++)
	{
		if (!initialized_ || event->x < x_start || event->x >= x_end || event->y < y_start || event->y >= y_end)
		{
			continue;
		}

		accumulation_start_timestamp_ =
			accumulation_start_timestamp_ == 0 ? event->timestamp : accumulation_start_timestamp_;

		if (temporal_sample_rate > 1)
		{
			const int64_t time_difference =
				static_cast<int64_t>(event->timestamp) - static_cast<int64_t>(accumulation_start_timestamp_);
			if (time_difference != 0)
			{
				if (time_difference > 0)
				{
					const int64_t rate = static_cast<int64_t>(temporal_sample_rate);
					accumulation_start_timestamp_ += ((time_difference + rate - 1) / rate) * rate;
				}
				continue;
			}
		}

		if (event_count_ >= event_capacity_ && !ensureEventCapacity())
		{
			continue;
		}

		const int local_x = static_cast<int>(event->x - x_start);
		const int local_y = static_cast<int>(event->y - y_start);
		const int sampled_x = spatial_rate_is_one ? local_x : static_cast<int>(local_x * spatial_rate_reciprocal);
		const int sampled_y = spatial_rate_is_one ? local_y : static_cast<int>(local_y * spatial_rate_reciprocal);

		Event &output_event = events_[event_count_];
		output_event.x = static_cast<uint16_t>(sampled_x);
		output_event.y = static_cast<uint16_t>(sampled_y);
		output_event.timestamp = event->timestamp;
		output_event.polarity = event->polarity;
		event_count_++;
	}
}

bool EventMemoryModule::ensureEventCapacity()
{
	if (event_count_ < event_capacity_)
	{
		return true;
	}

	struct stat shared_memory_stat;
	if (fstat(shared_memory_fd_, &shared_memory_stat) == -1)
	{
		RCLCPP_ERROR(node_->get_logger(), "Failed to get shared memory size: %s", strerror(errno));
		return false;
	}

	if (munmap((void *)events_, mapped_data_size_) == -1)
	{
		RCLCPP_ERROR(node_->get_logger(), "Failed to unmap shared memory: %s", strerror(errno));
		return false;
	}
	events_ = nullptr;

	reserved_data_multiplier_++;
	event_capacity_ = DATA_SIZE_PER_UINT * static_cast<size_t>(timespan_) * reserved_data_multiplier_;
	const size_t new_data_size = sizeof(Event) * event_capacity_;
	const int resize_result = ftruncate(shared_memory_fd_, new_data_size);
	if (resize_result == -1)
	{
		RCLCPP_ERROR(node_->get_logger(), "Failed to resize shared memory: %s", strerror(errno));
		initialized_ = false;
		event_capacity_ = 0;
		mapped_data_size_ = 0;
		return false;
	}

	void *new_memory = mmap(0, new_data_size, PROT_WRITE, MAP_SHARED, shared_memory_fd_, 0);
	if (new_memory == MAP_FAILED)
	{
		RCLCPP_ERROR(node_->get_logger(), "Failed to remap shared memory: %s", strerror(errno));
		initialized_ = false;
		event_capacity_ = 0;
		mapped_data_size_ = 0;
		return false;
	}

	events_ = static_cast<Event *>(new_memory);
	mapped_data_size_ = new_data_size;
	RCLCPP_DEBUG(node_->get_logger(), "Resized shared memory from %ld to %ld, success: %d",
				 shared_memory_stat.st_size, new_data_size, resize_result);
	return true;
}

void EventMemoryModule::publishTriggerWindow(const TriggerEvent &trigger, const rclcpp::Time &receive_stamp)
{
	if (!initialized_)
	{
		return;
	}

	{
		std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);
		captureReplayWindowState();
		if (target_num_events_per_sec_ != 0)
		{
			event_count_history_.push_front(interval_event_count_);
		}
		interval_event_count_ = 0;
	}

	TriggerPublishMetadata trigger_metadata;
	trigger_metadata.marked = true;
	trigger_metadata.id = static_cast<int16_t>(trigger.id);
	trigger_metadata.polarity = static_cast<int16_t>(trigger.polarity);
	trigger_metadata.timestamp = static_cast<int64_t>(trigger.timestamp);

	recordCurrentWindowReceiveStamp(receive_stamp);
	setReplayWindowOverride(static_cast<int64_t>(window_start_timestamp_), static_cast<int64_t>(trigger.timestamp),
							static_cast<int64_t>(trigger.timestamp - window_start_timestamp_), 1.0,
							trigger_metadata);
	publish();
	setWindowStartTimestamp(trigger.timestamp);
}

void EventMemoryModule::publishCurrentWindow()
{
	if (!initialized_)
	{
		std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);
		interval_event_count_ = 0;
		return;
	}

	{
		std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);
		captureReplayWindowState();
		if (target_num_events_per_sec_ != 0)
		{
			event_count_history_.push_front(interval_event_count_);
		}
		interval_event_count_ = 0;
	}

	publish();
}

void EventMemoryModule::setupMessage()
{
	if (!initialized_)
	{
		return;
	}

	std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);

	message_.identifier = shared_memory_file_;
	message_.length = event_count_;
	message_.header.frame_id = identifier_;
	message_.header.stamp = getWindowReceiveStamp();
	message_.spatial_sample_rate = spatial_sample_rate_;
	fillReplayWindow(message_.replay_window, region_of_interest_, 1.0, replay_window_temporal_sample_rate_,
					 temporal_sample_rate_min_, target_num_events_per_sec_,
					 replay_window_accumulation_start_timestamp_);
	if (ftruncate(shared_memory_fd_, sizeof(Event) * event_count_) == -1)
	{
		RCLCPP_ERROR(node_->get_logger(), "Failed to resize event shared memory: %s", strerror(errno));
		return;
	}

	if (events_ != nullptr)
	{
		if (munmap((void *)events_, mapped_data_size_) == -1)
		{
			RCLCPP_WARN(node_->get_logger(), "Unmap the data pointer has failed.");
		}
		events_ = nullptr;
	}

	auto data_size = sizeof(Event) * DATA_SIZE_PER_UINT * static_cast<size_t>(timespan_);
	events_ = static_cast<Event *>(createSharedMemory(data_size, -1));
	reserved_data_multiplier_ = 1;
	event_capacity_ = DATA_SIZE_PER_UINT * static_cast<size_t>(timespan_);
	mapped_data_size_ = data_size;
	initialized_ = events_ != nullptr;
	replay_window_state_captured_ = false;
}

void EventMemoryModule::captureReplayWindowState()
{
	if (replay_window_state_captured_)
	{
		return;
	}

	replay_window_accumulation_start_timestamp_ = accumulation_start_timestamp_;
	replay_window_temporal_sample_rate_ = temporal_sample_rate_;
	replay_window_state_captured_ = true;
}

void EventMemoryModule::adjust()
{
	std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);

	if (!initialized_)
	{
		return;
	}

	if (target_num_events_per_sec_ != 0)
	{
		if (timespan_ <= 0)
		{
			return;
		}

		const size_t target_window_size =
			std::max<size_t>(1, static_cast<size_t>(SEC_TO_MICROSEC / timespan_));
		if (event_count_history_.size() >= target_window_size)
		{
			double weighted_event_count = 0.0;
			const size_t history_size = event_count_history_.size();
			const size_t group_size = history_size / 3; // Split into 3 groups

			// Recent third
			double recent_sum =
				std::accumulate(event_count_history_.begin(), event_count_history_.begin() + group_size, 0.0);
			weighted_event_count += recent_sum * 1.5; // 60% weight to recent events

			// Middle third
			double middle_sum = std::accumulate(event_count_history_.begin() + group_size,
												event_count_history_.begin() + 2 * group_size, 0.0);
			weighted_event_count += middle_sum * 1.2; // 30% weight to middle events

			// Oldest third
			double oldest_sum =
				std::accumulate(event_count_history_.begin() + 2 * group_size, event_count_history_.end(), 0.0);
			weighted_event_count += oldest_sum * 0.3; // 10% weight to oldest events

			const double average_event_count = weighted_event_count / static_cast<double>(history_size);
			const double target_events_per_window =
				static_cast<double>(target_num_events_per_sec_) / static_cast<double>(target_window_size);
			if (target_events_per_window <= 0.0)
			{
				return;
			}

			const double sample_rate_scale = average_event_count / target_events_per_window;
			if (!std::isfinite(sample_rate_scale))
			{
				return;
			}
			if (sample_rate_scale > static_cast<double>(std::numeric_limits<int>::max()))
			{
				temporal_sample_rate_ = std::numeric_limits<int>::max();
			}
			else
			{
				temporal_sample_rate_ = std::max(temporal_sample_rate_min_, static_cast<int>(sample_rate_scale));
			}
			RCLCPP_DEBUG(node_->get_logger(), "Adjusted the temporal_sample_rate to %d", temporal_sample_rate_);

			if (event_count_history_.size() >= target_window_size)
			{
				event_count_history_.erase(event_count_history_.begin() + target_window_size,
										   event_count_history_.end());
			}
		}
	}
}
