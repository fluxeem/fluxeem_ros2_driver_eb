#pragma once

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <event_msgs/msg/replay_window.hpp>
#include <fcntl.h>
#include <fluxeem_ros2_publisher/openeb_compat.hpp>
#include <fluxeem_ros2_publisher/publisher_constant.h>
#include <iterator>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>

using EventCallback = void (*)(const Event *begin, const Event *end);

struct SharedMemory
{
	int shared_memory_fd_;
	std::string file_name_;

	SharedMemory(int shared_memory_fd, std::string file_name)
		: shared_memory_fd_(shared_memory_fd), file_name_(std::move(file_name))
	{
	}
};

struct RegionOfInterest
{
	unsigned short x_start_;
	unsigned short x_end_;
	unsigned short y_start_;
	unsigned short y_end_;
};

struct TriggerPublishMetadata
{
	bool marked{false};
	int16_t id{0};
	int16_t polarity{0};
	int64_t timestamp{0};
};

/// @brief Shared event-window mechanics for all event-derived publishers.
class WindowedEventModuleBase
{
  public:
	WindowedEventModuleBase(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name)
		: node_(node), identifier_(identifier), topic_name_(topic_name), event_count_(0), timespan_(0),
		  timestamp_cutoff_(0), initialized_(false), current_window_receive_stamp_valid_(false),
		  published_window_receive_stamp_valid_(false), window_start_timestamp_(0), replay_window_override_valid_(false),
		  replay_window_start_override_(0), replay_window_end_override_(0), replay_window_timespan_override_(0),
		  replay_window_keep_fraction_override_(1.0)
	{
	}

	virtual ~WindowedEventModuleBase() = default;

	virtual void reset() = 0;

	void setTimestampOrigin(Timestamp timestamp_origin)
	{
		if (timespan_ <= 0)
		{
			return;
		}

		timestamp_cutoff_ = timestamp_origin + static_cast<Timestamp>(timespan_);
		window_start_timestamp_ = timestamp_origin;
		onTimestampCutoffChanged();
	}

	void publishUntil(Timestamp timestamp)
	{
		if (!initialized_ || timespan_ <= 0)
		{
			return;
		}

		while (timestamp >= timestamp_cutoff_)
		{
			setReplayWindowOverride(window_start_timestamp_, timestamp_cutoff_, timestamp_cutoff_ - window_start_timestamp_,
									-1.0, TriggerPublishMetadata{});
			publishCurrentWindow();
			window_start_timestamp_ = timestamp_cutoff_;
			timestamp_cutoff_ += timespan_;
			onTimestampCutoffChanged();
		}
	}

	int getTimespan() const
	{
		return timespan_;
	}

	void processEvents(const Event *begin, const Event *end)
	{
		processEvents(begin, end, node_->now());
	}

	virtual void processEvents(const Event *begin, const Event *end, const rclcpp::Time &receive_stamp)
	{
		if (begin == end || !initialized_)
		{
			return;
		}

		const Event *current_begin = begin;
		const Timestamp end_timestamp = std::prev(end)->timestamp;
		while (current_begin != end)
		{
			if (current_begin->timestamp > timestamp_cutoff_)
			{
				setReplayWindowOverride(window_start_timestamp_, timestamp_cutoff_,
										timestamp_cutoff_ - window_start_timestamp_, -1.0,
										TriggerPublishMetadata{});
				publishCurrentWindow();
				window_start_timestamp_ = timestamp_cutoff_;
				advanceTimestampCutoff(current_begin->timestamp);
				window_start_timestamp_ = timestamp_cutoff_ - static_cast<Timestamp>(timespan_);
			}

			if (end_timestamp < timestamp_cutoff_)
			{
				recordCurrentWindowReceiveStamp(receive_stamp);
				processEventRange(current_begin, end);
				return;
			}

			const Event *current_end = std::upper_bound(current_begin, end, timestamp_cutoff_,
														[](const Timestamp &timestamp, const Event &event)
														{ return timestamp < event.timestamp; });
			recordCurrentWindowReceiveStamp(receive_stamp);
			processEventRange(current_begin, current_end);
			setReplayWindowOverride(window_start_timestamp_, timestamp_cutoff_,
									timestamp_cutoff_ - window_start_timestamp_, -1.0, TriggerPublishMetadata{});
			publishCurrentWindow();
			window_start_timestamp_ = timestamp_cutoff_;
			timestamp_cutoff_ += timespan_;
			onTimestampCutoffChanged();
			current_begin = current_end;
		}
	}

	void processEventsWithoutWindow(const Event *begin, const Event *end, const rclcpp::Time &receive_stamp)
	{
		if (begin == end || !initialized_)
		{
			return;
		}

		recordCurrentWindowReceiveStamp(receive_stamp);
		processEventRange(begin, end);
	}

	virtual void setTriggerWindowStart(Timestamp timestamp)
	{
		setWindowStartTimestamp(timestamp);
	}

	void publishTriggerWindow(Timestamp window_start, const TriggerEvent &trigger,
							  const rclcpp::Time &receive_stamp)
	{
		recordCurrentWindowReceiveStamp(receive_stamp);
		setReplayWindowOverride(static_cast<int64_t>(window_start), static_cast<int64_t>(trigger.timestamp),
								static_cast<int64_t>(trigger.timestamp - window_start), 1.0,
								makeTriggerMetadata(trigger));
		publishCurrentWindow();
		setTriggerWindowStart(trigger.timestamp);
	}

  protected:
	rclcpp::Node *node_;
	std::string identifier_;
	std::string topic_name_;
	size_t event_count_;
	int timespan_;
	Timestamp timestamp_cutoff_;
	bool initialized_;
	rclcpp::Time current_window_receive_stamp_;
	rclcpp::Time published_window_receive_stamp_;
	bool current_window_receive_stamp_valid_;
	bool published_window_receive_stamp_valid_;
	Timestamp window_start_timestamp_;
	bool replay_window_override_valid_;
	int64_t replay_window_start_override_;
	int64_t replay_window_end_override_;
	int64_t replay_window_timespan_override_;
	double replay_window_keep_fraction_override_;
	TriggerPublishMetadata replay_window_trigger_metadata_;

  protected:
	void configureTimeWindow(int timespan, const std::string &module_name)
	{
		if (timespan <= 0)
		{
			throw std::invalid_argument("The " + module_name + " timespan must be greater than 0");
		}

		timespan_ = timespan;
		resetTimeWindow();
	}

	void resetTimeWindow()
	{
		timestamp_cutoff_ = timespan_;
		window_start_timestamp_ = 0;
		current_window_receive_stamp_valid_ = false;
		published_window_receive_stamp_valid_ = false;
		clearReplayWindowOverride();
		onTimestampCutoffChanged();
	}

	void setWindowStartTimestamp(Timestamp timestamp)
	{
		window_start_timestamp_ = timestamp;
	}

	void recordCurrentWindowReceiveStamp(const rclcpp::Time &receive_stamp)
	{
		current_window_receive_stamp_ = receive_stamp;
		current_window_receive_stamp_valid_ = true;
	}

	void captureWindowReceiveStampForPublish()
	{
		// Preserve the batch arrival time before setupMessage() fills the ROS header.
		published_window_receive_stamp_ =
			current_window_receive_stamp_valid_ ? current_window_receive_stamp_ : node_->now();
		published_window_receive_stamp_valid_ = true;
		current_window_receive_stamp_valid_ = false;
	}

	rclcpp::Time getWindowReceiveStamp()
	{
		return published_window_receive_stamp_valid_ ? published_window_receive_stamp_ : node_->now();
	}

	virtual void processEventRange(const Event *begin, const Event *end)
	{
		if (begin == end)
		{
			return;
		}

		for (const Event *event = begin; event != end; event++)
		{
			processEvent(event);
		}
	}

	virtual void publishCurrentWindow() = 0;

	void advanceTimestampCutoff(Timestamp timestamp)
	{
		while (timestamp > timestamp_cutoff_)
		{
			timestamp_cutoff_ += timespan_;
		}
		onTimestampCutoffChanged();
	}

	virtual void onTimestampCutoffChanged()
	{
	}

	void setReplayWindowOverride(int64_t raw_window_start, int64_t raw_window_end, int64_t effective_timespan,
								 double keep_fraction, const TriggerPublishMetadata &trigger_metadata)
	{
		replay_window_override_valid_ = true;
		replay_window_start_override_ = raw_window_start;
		replay_window_end_override_ = raw_window_end;
		replay_window_timespan_override_ = effective_timespan;
		replay_window_keep_fraction_override_ = keep_fraction;
		replay_window_trigger_metadata_ = trigger_metadata;
	}

	void clearReplayWindowOverride()
	{
		replay_window_override_valid_ = false;
		replay_window_start_override_ = 0;
		replay_window_end_override_ = 0;
		replay_window_timespan_override_ = 0;
		replay_window_keep_fraction_override_ = 1.0;
		replay_window_trigger_metadata_ = TriggerPublishMetadata{};
	}

	static TriggerPublishMetadata makeTriggerMetadata(const TriggerEvent &trigger)
	{
		TriggerPublishMetadata trigger_metadata;
		trigger_metadata.marked = true;
		trigger_metadata.id = static_cast<int16_t>(trigger.id);
		trigger_metadata.polarity = static_cast<int16_t>(trigger.polarity);
		trigger_metadata.timestamp = static_cast<int64_t>(trigger.timestamp);
		return trigger_metadata;
	}

	virtual void processEvent(const Event *event_ptr) = 0;
};

/// @brief The base class for the event topic module
/// @tparam MsgType the type of the message
template <typename MsgType> class EventTopicModule : public WindowedEventModuleBase
{
  public:
	/// @brief The constructor of the event topic module
	/// @param node the pointer to the node
	/// @param identifier the identifier of the module
	/// @param topic_name the topic name of the module
	/// @param history_depth the history depth of the module
	/// @param lifetime the lifetime of the module, in seconds
	EventTopicModule(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name,
					 size_t history_depth, double lifetime)
		: WindowedEventModuleBase(node, identifier, topic_name), message_count_(0), shared_memory_fd_(-1),
		  lifetime_(rclcpp::Duration::from_seconds(lifetime)), raw_file_start_timestamp_(-1)
	{
		publisher_ = node->create_publisher<MsgType>(identifier + "/" + topic_name,
													 rclcpp::QoS(rclcpp::KeepLast(history_depth)));
	}

	/// @brief The destructor of the event topic module
	virtual ~EventTopicModule()
	{
		cleanSharedMemory(true);
	}

	/// @brief The function to publish the message
	void publish()
	{
		if (!initialized_)
		{
			return;
		}

		shared_memory_time_queue_.push(node_->now());
		captureWindowReceiveStampForPublish();
		message_count_++;

		setupMessage();
		publisher_->publish(message_);
		event_count_ = 0;
		published_window_receive_stamp_valid_ = false;
		clearReplayWindowOverride();
		cleanSharedMemory(false);
	}

	/// @brief clean the shared memory which reached its EOL
	/// @param clean_all
	void cleanSharedMemory(bool clean_all)
	{
		std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);
		try
		{
			if (clean_all)
			{
				RCLCPP_INFO(node_->get_logger(), "Cleaning all the shared memory");
				while (!shared_memory_queue_.empty())
				{
					close(shared_memory_queue_.front().shared_memory_fd_);
					shm_unlink(shared_memory_queue_.front().file_name_.c_str());
					shared_memory_queue_.pop();
				}
				std::queue<rclcpp::Time>().swap(shared_memory_time_queue_);
				cleanOwnedSharedMemoryFiles();
			}
			else
			{
				rclcpp::Time cur_time = node_->now();
				while (!shared_memory_queue_.empty() && !shared_memory_time_queue_.empty() &&
					   (cur_time - shared_memory_time_queue_.front() > lifetime_))
				{
					close(shared_memory_queue_.front().shared_memory_fd_);
					shm_unlink(shared_memory_queue_.front().file_name_.c_str());
					shared_memory_queue_.pop();
					shared_memory_time_queue_.pop();
				}
			}
		}
		catch (std::exception &e)
		{
			RCLCPP_ERROR(node_->get_logger(), "Failed to clean shared memory, the error is %s", e.what());
		}
	}

	void setReplaySourceMetadata(const std::string &raw_file_path, const std::string &raw_file_hash,
								 int64_t raw_file_start_timestamp)
	{
		raw_file_path_ = raw_file_path;
		raw_file_hash_ = raw_file_hash;
		raw_file_start_timestamp_ = raw_file_start_timestamp;
	}

  protected:
	typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
	MsgType message_;
	size_t message_count_;

	std::string shared_memory_file_;
	int shared_memory_fd_;
	std::queue<SharedMemory> shared_memory_queue_;
	std::queue<rclcpp::Time> shared_memory_time_queue_;
	std::shared_mutex shared_memory_mutex_;

	rclcpp::Duration lifetime_;
	std::string raw_file_path_;
	std::string raw_file_hash_;
	int64_t raw_file_start_timestamp_;

  protected:
	void processEventRange(const Event *begin, const Event *end) override
	{
		if (begin == end)
		{
			return;
		}

		std::lock_guard<std::shared_mutex> lock(shared_memory_mutex_);
		for (const Event *event = begin; event != end; event++)
		{
			processEvent(event);
		}
	}

	void publishCurrentWindow() override
	{
		publish();
	}

	// ReplayWindow stores camera-side us timestamps; ROS headers use receive stamps.
	void fillReplayWindow(event_msgs::msg::ReplayWindow &replay_window, const RegionOfInterest &region_of_interest,
						  double keep_fraction, int temporal_sample_rate = 1, int temporal_sample_rate_min = 1,
						  uint64_t target_num_events_per_sec = 0, int64_t accumulation_start_timestamp = 0) const
	{
		replay_window.valid = true;
		replay_window.raw_file_path = raw_file_path_;
		replay_window.raw_file_hash = raw_file_hash_;
		replay_window.raw_file_start_timestamp = raw_file_start_timestamp_;
		replay_window.raw_window_start =
			replay_window_override_valid_ ? replay_window_start_override_ : static_cast<int64_t>(window_start_timestamp_);
		replay_window.raw_window_end =
			replay_window_override_valid_ ? replay_window_end_override_ : static_cast<int64_t>(timestamp_cutoff_);
		replay_window.roi_x_start = region_of_interest.x_start_;
		replay_window.roi_x_end = region_of_interest.x_end_;
		replay_window.roi_y_start = region_of_interest.y_start_;
		replay_window.roi_y_end = region_of_interest.y_end_;
		replay_window.timespan =
			static_cast<int32_t>(replay_window_override_valid_ ? replay_window_timespan_override_ : timespan_);
		replay_window.keep_fraction =
			replay_window_override_valid_ && replay_window_keep_fraction_override_ >= 0.0
				? replay_window_keep_fraction_override_
				: keep_fraction;
		replay_window.temporal_sample_rate = temporal_sample_rate;
		replay_window.temporal_sample_rate_min = temporal_sample_rate_min;
		replay_window.target_num_events_per_sec = target_num_events_per_sec;
		replay_window.accumulation_start_timestamp = accumulation_start_timestamp;
		replay_window.trigger_marked = replay_window_trigger_metadata_.marked;
		replay_window.trigger_id = replay_window_trigger_metadata_.id;
		replay_window.trigger_polarity = replay_window_trigger_metadata_.polarity;
		replay_window.trigger_timestamp = replay_window_trigger_metadata_.timestamp;
	}

	/// @brief create the shared memory
	/// @param data_size the size of the shared memory
	/// @param value the value to be set to the shared memory, 0 - 255, -1 for no value setting
	/// @return the pointer to the shared memory
	void *createSharedMemory(size_t data_size, int value)
	{
		std::string next_shared_memory_file = makeSharedMemoryName();
		int next_shared_memory_fd = shm_open(next_shared_memory_file.c_str(), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
		if (next_shared_memory_fd == -1)
		{
			RCLCPP_ERROR(node_->get_logger(), "Failed to create shared memory: %s", strerror(errno));
			return nullptr;
		}

		if (ftruncate(next_shared_memory_fd, data_size) == -1)
		{
			RCLCPP_ERROR(node_->get_logger(), "Failed to resize shared memory: %s", strerror(errno));
			close(next_shared_memory_fd);
			shm_unlink(next_shared_memory_file.c_str());
			return nullptr;
		}

		void *mapped_memory = mmap(0, data_size, PROT_WRITE, MAP_SHARED, next_shared_memory_fd, 0);
		if (mapped_memory == MAP_FAILED)
		{
			RCLCPP_ERROR(node_->get_logger(), "Failed to map shared memory: %s", strerror(errno));
			close(next_shared_memory_fd);
			shm_unlink(next_shared_memory_file.c_str());
			return nullptr;
		}

		if (value >= 0 && value <= 255)
		{
			memset(mapped_memory, value, data_size);
		}

		shared_memory_file_ = next_shared_memory_file;
		shared_memory_fd_ = next_shared_memory_fd;
		shared_memory_queue_.push(SharedMemory(shared_memory_fd_, shared_memory_file_));

		return mapped_memory;
	}

	std::string makeSharedMemoryName() const
	{
		std::string shared_memory_name = makeSharedMemoryPrefix() + std::to_string(message_count_);
		return "/" + shared_memory_name;
	}

	std::string makeSharedMemoryPrefix() const
	{
		std::string shared_memory_name = identifier_ + "_" + topic_name_ + "_idx";
		std::replace(shared_memory_name.begin(), shared_memory_name.end(), '/', '_');
		return shared_memory_name;
	}

	void cleanOwnedSharedMemoryFiles()
	{
		DIR *shared_memory_dir = opendir("/dev/shm");
		if (shared_memory_dir == nullptr)
		{
			RCLCPP_WARN(node_->get_logger(), "Failed to open /dev/shm while cleaning shared memory: %s",
						strerror(errno));
			return;
		}

		const std::string shared_memory_prefix = makeSharedMemoryPrefix();
		while (dirent *entry = readdir(shared_memory_dir))
		{
			const std::string file_name(entry->d_name);
			if (file_name.rfind(shared_memory_prefix, 0) != 0)
			{
				continue;
			}

			const std::string shared_memory_name = "/" + file_name;
			if (shm_unlink(shared_memory_name.c_str()) == -1 && errno != ENOENT)
			{
				RCLCPP_WARN(node_->get_logger(), "Failed to unlink shared memory %s: %s", shared_memory_name.c_str(),
							strerror(errno));
			}
		}

		closedir(shared_memory_dir);
	}

	/// @brief The pure virtual function to setup the message
	virtual void setupMessage() = 0;

	/// @brief The pure virtual function to process the event
	/// @param event_ptr the pointer to the event
	virtual void processEvent(const Event *event_ptr) = 0;
};
