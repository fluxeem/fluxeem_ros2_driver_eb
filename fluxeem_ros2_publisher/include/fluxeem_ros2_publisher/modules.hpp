#pragma once

#include "event_topic_module.hpp"
#include <cmath>
#include <deque>
#include <event_msgs/msg/event_memory.hpp>
#include <event_msgs/msg/histogram_memory.hpp>
#include <event_msgs/msg/time_surface_memory.hpp>
#include <sensor_msgs/msg/image.hpp>

/// @brief The module for event memory
class EventMemoryModule : public EventTopicModule<event_msgs::msg::EventMemory>
{
  public:
	EventMemoryModule(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name,
					  size_t history_depth, double lifetime);
	~EventMemoryModule() override;
	void setupModule(RegionOfInterest region_of_interest, double spatial_sample_rate, int temporal_sample_rate_min,
					 size_t target_num_events_per_sec, int timespan);
	void reset() override;
	void processEvent(const Event *event_ptr) override;
	void publishTriggerWindow(const TriggerEvent &trigger, const rclcpp::Time &receive_stamp);

  private:
	Event *events_;
	RegionOfInterest region_of_interest_;
	double spatial_sample_rate_;
	double spatial_sample_rate_reciprocal_;
	bool spatial_sample_rate_is_one_;
	int temporal_sample_rate_;
	int temporal_sample_rate_min_;
	size_t target_num_events_per_sec_;
	size_t interval_event_count_;
	long long accumulation_start_timestamp_;
	long long replay_window_accumulation_start_timestamp_;
	int replay_window_temporal_sample_rate_;
	bool replay_window_state_captured_;
	rclcpp::TimerBase::SharedPtr adjust_timer_;
	std::deque<std::size_t> event_count_history_;
	uint reserved_data_multiplier_;
	size_t event_capacity_;
	size_t mapped_data_size_;

  private:
	void processEventRange(const Event *begin, const Event *end) override;
	void publishCurrentWindow() override;
	void setupMessage() override;

	bool ensureEventCapacity();
	void captureReplayWindowState();
	void adjust();
};

template <typename MsgType> class ImageMemoryModuleBase : public EventTopicModule<MsgType>
{
  public:
	ImageMemoryModuleBase(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name,
						  size_t history_depth, double lifetime, std::string module_name)
		: EventTopicModule<MsgType>(node, identifier, topic_name, history_depth, lifetime), data_(nullptr),
		  channel_(0), width_(0), height_(0), keep_fraction_(1.0), start_timestamp_(0),
		  module_name_(std::move(module_name))
	{
	}

	~ImageMemoryModuleBase() override
	{
		std::lock_guard<std::shared_mutex> lock(this->shared_memory_mutex_);
		unmapBuffer();
		RCLCPP_INFO(this->node_->get_logger(), "Cleaning the shared memory of %s", module_name_.c_str());
	}

	void reset() override
	{
		std::lock_guard<std::shared_mutex> lock(this->shared_memory_mutex_);
		this->event_count_ = 0;
		this->resetTimeWindow();
		fillBuffer();
	}

	void setupModule(RegionOfInterest region_of_interest, int channel, double keep_fraction, int timespan);

	void setTriggerWindowStart(Timestamp timestamp) override
	{
		start_timestamp_ = timestamp;
		this->setWindowStartTimestamp(timestamp);
	}

  protected:
	uint8_t *data_;
	int channel_;
	int width_;
	int height_;
	RegionOfInterest region_of_interest_;
	double keep_fraction_;
	Timestamp start_timestamp_;
	std::string module_name_;

  private:
	void onTimestampCutoffChanged() override;
	void setupMessage() override;
	void unmapBuffer();
	void fillBuffer();
	size_t dataSize() const;
	virtual int initialValue() const = 0;
};

template <typename MsgType>
void ImageMemoryModuleBase<MsgType>::setupModule(RegionOfInterest region_of_interest, int channel,
												 double keep_fraction, int timespan)
{
	region_of_interest_ = region_of_interest;
	width_ = region_of_interest.x_end_ - region_of_interest.x_start_;
	height_ = region_of_interest.y_end_ - region_of_interest.y_start_;
	channel_ = channel;
	if (channel_ != 1 && channel_ != 3)
	{
		throw std::invalid_argument("The channel of the " + module_name_ + " must be 1 or 3");
	}
	keep_fraction_ = keep_fraction;
	if (!std::isfinite(keep_fraction_) || keep_fraction_ < 0 || keep_fraction_ > 1)
	{
		throw std::invalid_argument("The keep fraction must be between 0 and 1");
	}
	this->configureTimeWindow(timespan, module_name_);

	this->message_.width = width_;
	this->message_.height = height_;
	this->message_.channel = channel_;

	std::lock_guard<std::shared_mutex> lock(this->shared_memory_mutex_);
	data_ = static_cast<uint8_t *>(this->createSharedMemory(dataSize(), initialValue()));
	this->initialized_ = data_ != nullptr;
}

template <typename MsgType> void ImageMemoryModuleBase<MsgType>::onTimestampCutoffChanged()
{
	start_timestamp_ =
		this->timestamp_cutoff_ - static_cast<Timestamp>(static_cast<double>(this->timespan_) * keep_fraction_);
}

template <typename MsgType> void ImageMemoryModuleBase<MsgType>::setupMessage()
{
	this->message_.identifier = this->shared_memory_file_;
	this->message_.header.frame_id = this->identifier_;
	this->message_.header.stamp = this->getWindowReceiveStamp();
	this->fillReplayWindow(this->message_.replay_window, region_of_interest_, keep_fraction_);

	std::lock_guard<std::shared_mutex> lock(this->shared_memory_mutex_);
	unmapBuffer();
	data_ = static_cast<uint8_t *>(this->createSharedMemory(dataSize(), initialValue()));
	this->initialized_ = data_ != nullptr;
}

template <typename MsgType> void ImageMemoryModuleBase<MsgType>::unmapBuffer()
{
	if (data_ == nullptr)
	{
		return;
	}

	if (munmap(static_cast<void *>(data_), dataSize()) == -1)
	{
		RCLCPP_WARN(this->node_->get_logger(), "Unmap the data pointer has failed.");
	}
	data_ = nullptr;
}

template <typename MsgType> void ImageMemoryModuleBase<MsgType>::fillBuffer()
{
	if (data_ != nullptr)
	{
		memset(data_, initialValue(), dataSize());
	}
}

template <typename MsgType> size_t ImageMemoryModuleBase<MsgType>::dataSize() const
{
	return sizeof(uint8_t) * width_ * height_ * channel_;
}

/// @brief The module for time surface memory
class TimeSurfaceMemoryModule : public ImageMemoryModuleBase<event_msgs::msg::TimeSurfaceMemory>
{
  public:
	TimeSurfaceMemoryModule(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name,
							size_t history_depth, double lifetime);

  private:
	int initialValue() const override;
	void processEvent(const Event *event_ptr) override;
	void processEventRange(const Event *begin, const Event *end) override;
};

/// @brief The module for histogram memory
class HistogramMemoryModule : public ImageMemoryModuleBase<event_msgs::msg::HistogramMemory>
{
  public:
	HistogramMemoryModule(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name,
						  size_t history_depth, double lifetime);

  private:
	int initialValue() const override;
	void processEvent(const Event *event_ptr) override;
	void processEventRange(const Event *begin, const Event *end) override;
};

class EventToImageModule : public WindowedEventModuleBase
{
  public:
	EventToImageModule(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name,
					   size_t history_depth);
	~EventToImageModule() override;

	void reset() override;
	void setupModule(const std::string &image_type, RegionOfInterest region_of_interest, int channel,
					 double keep_fraction, int timespan);
	void processEvent(const Event *event_ptr) override;
	void setTriggerWindowStart(Timestamp timestamp) override;

	void publish();

  private:
	void publishCurrentWindow() override;
	void onTimestampCutoffChanged() override;
	void processEventRange(const Event *begin, const Event *end) override;
	void updateStartTimestamp();
	void resetImageMessage();
	void setupMessage();

  private:
	std::string image_type_;
	RegionOfInterest region_of_interest_;
	double keep_fraction_;
	Timestamp start_timestamp_;

	int width_;
	int height_;
	int channel_;

	std::unique_ptr<sensor_msgs::msg::Image> image_message_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};
