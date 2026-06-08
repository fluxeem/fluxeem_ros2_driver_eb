#include "fluxeem_ros2_publisher/modules.hpp"
#include "fluxeem_ros2_publisher/pixel_operations.hpp"
#include <cmath>
#include <sensor_msgs/image_encodings.hpp>
#include <stdexcept>

EventToImageModule::EventToImageModule(rclcpp::Node *node, const std::string &identifier, const std::string &topic_name,
									   size_t history_depth)
	: WindowedEventModuleBase(node, identifier, topic_name), image_type_("time_surface"), keep_fraction_(1.0),
	  start_timestamp_(0), width_(0), height_(0), channel_(1)
{
	publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(identifier + "/" + topic_name,
																  rclcpp::QoS(rclcpp::KeepLast(history_depth)));
}

EventToImageModule::~EventToImageModule()
{
}

void EventToImageModule::reset()
{
	resetTimeWindow();
	resetImageMessage();
}

void EventToImageModule::setupModule(const std::string &image_type, RegionOfInterest region_of_interest, int channel,
									 double keep_fraction, int timespan)
{
	image_type_ = image_type;
	if (image_type_ != "time_surface" && image_type_ != "histogram")
	{
		throw std::invalid_argument("The image type must be time_surface or histogram");
	}
	region_of_interest_ = region_of_interest;
	keep_fraction_ = keep_fraction;
	if (!std::isfinite(keep_fraction_) || keep_fraction_ < 0 || keep_fraction_ > 1)
	{
		throw std::invalid_argument("The keep fraction must be between 0 and 1");
	}
	configureTimeWindow(timespan, "event to image module");

	width_ = region_of_interest.x_end_ - region_of_interest.x_start_;
	height_ = region_of_interest.y_end_ - region_of_interest.y_start_;
	channel_ = channel;
	if (channel_ != 1 && channel_ != 3)
	{
		throw std::invalid_argument("The channel must be 1 or 3");
	}

	resetImageMessage();
	initialized_ = true;
}

void EventToImageModule::processEvent(const Event *event_ptr)
{
	processEventRange(event_ptr, event_ptr + 1);
}

void EventToImageModule::setTriggerWindowStart(Timestamp timestamp)
{
	start_timestamp_ = timestamp;
	setWindowStartTimestamp(timestamp);
}

void EventToImageModule::publish()
{
	if (image_message_ && initialized_)
	{
		captureWindowReceiveStampForPublish();
		setupMessage();

		publisher_->publish(std::move(image_message_));
		published_window_receive_stamp_valid_ = false;
		clearReplayWindowOverride();

		resetImageMessage();
	}
}

void EventToImageModule::publishCurrentWindow()
{
	publish();
}

void EventToImageModule::processEventRange(const Event *begin, const Event *end)
{
	if (begin == end || !initialized_ || !image_message_)
	{
		return;
	}

	uint8_t *data = image_message_->data.data();
	if (image_type_ == "time_surface")
	{
		pixel_operations::stackTimeSurfaceRange(data, begin, end, start_timestamp_, region_of_interest_, width_,
												channel_);
	}
	else if (image_type_ == "histogram")
	{
		pixel_operations::stackHistogramRange(data, begin, end, start_timestamp_, region_of_interest_, width_, channel_);
	}
}

void EventToImageModule::onTimestampCutoffChanged()
{
	updateStartTimestamp();
}

void EventToImageModule::updateStartTimestamp()
{
	start_timestamp_ =
		timestamp_cutoff_ - static_cast<Timestamp>(static_cast<double>(timespan_) * keep_fraction_);
}

void EventToImageModule::resetImageMessage()
{
	image_message_ = std::make_unique<sensor_msgs::msg::Image>();
	if (channel_ == 1)
	{
		image_message_->data.resize(width_ * height_ * channel_, BG_COLOR);
	}
	else if (channel_ == 3)
	{
		image_message_->data.resize(width_ * height_ * channel_, 0);
	}
}

void EventToImageModule::setupMessage()
{
	image_message_->header.frame_id = identifier_;
	image_message_->header.stamp = getWindowReceiveStamp();
	image_message_->width = width_;
	image_message_->height = height_;
	image_message_->step = width_ * channel_;
	image_message_->encoding =
		(channel_ == 1) ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
}
