#include <fluxeem_ros2_publisher/fluxeem_ros2_publisher.h>

#include <cmath>
#include <stdexcept>

namespace
{
void validatePositiveInt(const std::string &name, int value)
{
	if (value <= 0)
	{
		throw std::invalid_argument(name + " must be greater than 0");
	}
}

void validateNonNegativeInt(const std::string &name, int value)
{
	if (value < 0)
	{
		throw std::invalid_argument(name + " must be greater than or equal to 0");
	}
}

void validateFraction(const std::string &name, double value)
{
	if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
	{
		throw std::invalid_argument(name + " must be finite and in the range (0, 1]");
	}
}

void validateUnitInterval(const std::string &name, double value)
{
	if (!std::isfinite(value) || value < 0.0 || value > 1.0)
	{
		throw std::invalid_argument(name + " must be finite and in the range [0, 1]");
	}
}
} // namespace

template <class T>
void FluxeemRos2Publisher::declareParam(const std::string &name, const T &default_value, const std::string &description)
{
	rcl_interfaces::msg::ParameterDescriptor desc;
	desc.description = description;
	this->declare_parameter(name, default_value, desc);
}

void FluxeemRos2Publisher::setGeneralParams()
{
	declareParam<std::string>("identifier", "fluxeem",
							  "A string as an identifier, which helps distinguish from others");
	identifier_ = get_parameter("identifier").as_string();

	declareParam<std::string>("raw_file", "", "The path of .raw file, then the camera will be read from the .raw file");
	raw_file_ = get_parameter("raw_file").as_string();

	declareParam<int>("raw_playback.interval_us", 0,
					  "The raw playback input slice in us. Use the minimum enabled module timespan when set to 0");
	raw_playback_interval_us_ = get_parameter("raw_playback.interval_us").as_int();
	validateNonNegativeInt("raw_playback.interval_us", raw_playback_interval_us_);

	declareParam<double>("raw_playback.rate", 1.0, "The playback speed for raw file input");
	raw_playback_rate_ = get_parameter("raw_playback.rate").as_double();
	if (!std::isfinite(raw_playback_rate_) || raw_playback_rate_ <= 0.0)
	{
		RCLCPP_WARN(this->get_logger(), "Invalid raw playback rate %.3f, using 1.0", raw_playback_rate_);
		raw_playback_rate_ = 1.0;
	}

	declareParam<bool>("raw_playback.loop", false, "Whether to loop raw file playback after reaching EOF");
	raw_playback_loop_ = get_parameter("raw_playback.loop").as_bool();

	setPublishStrategyParams();
}

void FluxeemRos2Publisher::setPublishStrategyParams()
{
	declareParam<std::string>("publish_strategy.mode", "timespan",
							  "The publish strategy. Supported values are timespan and trigger");
	const std::string publish_strategy_mode = get_parameter("publish_strategy.mode").as_string();
	if (publish_strategy_mode == "trigger")
	{
		trigger_publish_mode_ = true;
	}
	else if (publish_strategy_mode == "timespan")
	{
		trigger_publish_mode_ = false;
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "Invalid publish_strategy.mode '%s', using timespan",
					publish_strategy_mode.c_str());
		trigger_publish_mode_ = false;
	}

	declareParam<int>("publish_strategy.trigger_polarity", 1,
					  "Trigger polarity used by trigger publish mode");
	trigger_polarity_ = get_parameter("publish_strategy.trigger_polarity").as_int();
}

void FluxeemRos2Publisher::setCameraParams()
{
	declareParam<double>("roi.width_fraction", 1.0, "The fraction of the width of the RegionOfInterest");
	width_fraction_ = get_parameter("roi.width_fraction").as_double();

	declareParam<double>("roi.height_fraction", 1.0, "The fraction of the height of the RegionOfInterest");
	height_fraction_ = get_parameter("roi.height_fraction").as_double();
	validateFraction("roi.width_fraction", width_fraction_);
	validateFraction("roi.height_fraction", height_fraction_);

	uint16_t camera_height = 0;
	uint16_t camera_width = 0;
	std::string camera_serial;
	if (isRawFileMode())
	{
		camera_height = raw_height_;
		camera_width = raw_width_;
		camera_serial = raw_file_;
	}
	else
	{
		const auto &camera_config = camera_->get_camera_configuration();
		camera_height = static_cast<uint16_t>(camera_->geometry().get_height());
		camera_width = static_cast<uint16_t>(camera_->geometry().get_width());
		camera_serial = camera_config.serial_number;
	}

	unsigned short x_start = (1 - width_fraction_) / 2 * camera_width;
	unsigned short y_start = (1 - height_fraction_) / 2 * camera_height;
	unsigned short x_end = x_start + width_fraction_ * camera_width;
	unsigned short y_end = y_start + height_fraction_ * camera_height;
	region_of_interest_.x_start_ = x_start;
	region_of_interest_.x_end_ = x_end;
	region_of_interest_.y_start_ = y_start;
	region_of_interest_.y_end_ = y_end;

	RCLCPP_INFO(this->get_logger(), "[CONF] Width:%i, Height:%i", x_end - x_start, y_end - y_start);
	RCLCPP_INFO(this->get_logger(), "[CONF] Source: %s", camera_serial.c_str());

	camera_info_.width = uint32_t(x_end - x_start);
	camera_info_.height = uint32_t(y_end - y_start);
	camera_info_.header.frame_id = identifier_;

	declareParam<std::string>("camera.trail_filter_type", "STC_CUT_TRAIL", "The trail filter mode");
	std::string trail_filter_type = get_parameter("camera.trail_filter_type").as_string();

	declareParam<int>("camera.trail_filter_threshold", 50, "The trail filter threshold");
	int trail_filter_threshold = get_parameter("camera.trail_filter_threshold").as_int();
	if (trail_filter_threshold < 1 || trail_filter_threshold > 100)
	{
		trail_filter_threshold = 50;
	}

	declareParam<bool>("camera.trail_filter_enabled", false, "Whether to enable trail filter");
	bool trail_filter_enabled = get_parameter("camera.trail_filter_enabled").as_bool();
	if (!raw_file_.empty())
	{
		trail_filter_enabled = false;
	}

	if (!camera_settings_.trail_filter_.setParams(trail_filter_enabled, trail_filter_threshold, trail_filter_type))
	{
		throw std::invalid_argument("camera.trail_filter_type must be STC_CUT_TRAIL, STC_KEEP_TRAIL, or TRAIL");
	}

	declareParam<bool>("camera.erc_enabled", false, "Whether to enable event rate control");
	bool event_rate_control_enabled = get_parameter("camera.erc_enabled").as_bool();

	declareParam<int>("camera.erc_rate", 45, "The maximum event rate, in units of Mev/s");
	int event_rate_control_rate = get_parameter("camera.erc_rate").as_int();
	validateNonNegativeInt("camera.erc_rate", event_rate_control_rate);

	if (event_rate_control_enabled)
	{
		RCLCPP_INFO(this->get_logger(), "[CONF] Event rate control: %d", event_rate_control_rate);
	}
	camera_settings_.event_rate_control_.setParams(event_rate_control_enabled, event_rate_control_rate);

	declareParam<int>("camera.bias_diff", 0, "The difference of bias for the diff");
	int bias_diff = get_parameter("camera.bias_diff").as_int();

	declareParam<int>("camera.bias_diff_on", 0, "The difference of bias for the diff on");
	int bias_diff_on = get_parameter("camera.bias_diff_on").as_int();

	declareParam<int>("camera.bias_diff_off", 0, "The difference of bias for the diff off");
	int bias_diff_off = get_parameter("camera.bias_diff_off").as_int();

	declareParam<int>("camera.bias_fo", 0, "The difference of bias for the fo");
	int bias_fo = get_parameter("camera.bias_fo").as_int();

	declareParam<int>("camera.bias_hpf", 0, "The difference of bias for the hpf");
	int bias_hpf = get_parameter("camera.bias_hpf").as_int();

	declareParam<int>("camera.bias_refr", 0, "The difference of bias for the refr");
	int bias_refr = get_parameter("camera.bias_refr").as_int();

	camera_settings_.biases_.setParams(bias_diff, bias_diff_on, bias_diff_off, bias_fo, bias_hpf, bias_refr);
}

void FluxeemRos2Publisher::setEventMemoryParams()
{
	declareParam<bool>("event_memory_module.on_off", true, "Whether to enable the event memory module");
	bool event_memory_on = get_parameter("event_memory_module.on_off").as_bool();

	if (!event_memory_on)
	{
		return;
	}

	declareParam<std::string>("event_memory_module.topic_name", "event_memory", "The topic name for the event memory");
	std::string topic_event_memory = get_parameter("event_memory_module.topic_name").as_string();

	declareParam<int>("event_memory_module.timespan", 33333,
					  "The publish interval for the event memory module, the unit is us");
	int event_memory_timespan = get_parameter("event_memory_module.timespan").as_int();
	validatePositiveInt("event_memory_module.timespan", event_memory_timespan);
	updateMinModuleTimespan(event_memory_timespan);

	declareParam<double>("event_memory_module.spatial_sample_rate", 1,
						 "The sub-sample rate for obtaining events in spatial domain");
	double spatial_sample_rate = get_parameter("event_memory_module.spatial_sample_rate").as_double();
	if (!std::isfinite(spatial_sample_rate) || spatial_sample_rate < 1.0)
	{
		throw std::invalid_argument("event_memory_module.spatial_sample_rate must be finite and greater than or equal to 1");
	}

	declareParam<int>("event_memory_module.temporal_sample_rate_min", 1,
					  "The sub-sample rate for obtaining events in temporal domain");
	int temporal_sample_rate_min = get_parameter("event_memory_module.temporal_sample_rate_min").as_int();
	validatePositiveInt("event_memory_module.temporal_sample_rate_min", temporal_sample_rate_min);

	declareParam<int>("event_memory_module.target_num_events_per_sec", 360000,
					  "This number controls the final target number of events written in shared memory");
	int target_num_events_per_sec = get_parameter("event_memory_module.target_num_events_per_sec").as_int();
	validateNonNegativeInt("event_memory_module.target_num_events_per_sec", target_num_events_per_sec);

	event_memory_module_ =
		std::make_unique<EventMemoryModule>(this, identifier_, topic_event_memory, QOS_QUEUE, CLEAN_TIME_SPAN_SEC);
	event_memory_module_->setupModule(region_of_interest_, spatial_sample_rate, temporal_sample_rate_min,
									  target_num_events_per_sec, event_memory_timespan);
}

void FluxeemRos2Publisher::setHistogramMemoryParams()
{
	declareParam<bool>("histogram_memory_module.on_off", true, "Whether to enable the histogram memory module");
	bool histogram_memory_on = get_parameter("histogram_memory_module.on_off").as_bool();

	if (!histogram_memory_on)
	{
		return;
	}

	declareParam<std::string>("histogram_memory_module.topic_name", "histogram_memory",
							  "The topic name for the histogram memory");
	std::string topic_histogram_memory = get_parameter("histogram_memory_module.topic_name").as_string();

	declareParam<int>("histogram_memory_module.timespan", 33333,
					  "The publish interval for the histogram memory module, the unit is us");
	int histogram_memory_timespan = get_parameter("histogram_memory_module.timespan").as_int();
	validatePositiveInt("histogram_memory_module.timespan", histogram_memory_timespan);
	updateMinModuleTimespan(histogram_memory_timespan);

	declareParam<int>("histogram_memory_module.channel", 1, "The number of channel for histogram, only support 1 to 3");
	int channel = get_parameter("histogram_memory_module.channel").as_int();

	declareParam<double>("histogram_memory_module.keep_fraction", 0.5,
						 "The fraction of the time surface memory to keep");
	double keep_fraction = get_parameter("histogram_memory_module.keep_fraction").as_double();
	validateUnitInterval("histogram_memory_module.keep_fraction", keep_fraction);

	histogram_memory_module_ = std::make_unique<HistogramMemoryModule>(this, identifier_, topic_histogram_memory,
																	   QOS_QUEUE, CLEAN_TIME_SPAN_SEC);
	histogram_memory_module_->setupModule(region_of_interest_, channel, keep_fraction, histogram_memory_timespan);
}

void FluxeemRos2Publisher::setTimeSurfaceMemoryParams()
{
	declareParam<bool>("time_surface_memory_module.on_off", true, "Whether to enable the time surface memory module");
	bool time_surface_memory_on = get_parameter("time_surface_memory_module.on_off").as_bool();

	if (!time_surface_memory_on)
	{
		return;
	}

	declareParam<std::string>("time_surface_memory_module.topic_name", "time_surface_memory",
							  "The topic name for the time surface memory");
	std::string topic_time_surface_memory = get_parameter("time_surface_memory_module.topic_name").as_string();

	declareParam<int>("time_surface_memory_module.timespan", 33333,
					  "The publish interval for the time surface memory module, the unit is us");
	int time_surface_memory_timespan = get_parameter("time_surface_memory_module.timespan").as_int();
	validatePositiveInt("time_surface_memory_module.timespan", time_surface_memory_timespan);
	updateMinModuleTimespan(time_surface_memory_timespan);

	declareParam<int>("time_surface_memory_module.channel", 1,
					  "The number of channel for time surface, only support 1 to 3");
	int channel = get_parameter("time_surface_memory_module.channel").as_int();

	declareParam<double>("time_surface_memory_module.keep_fraction", 0.5,
						 "The fraction of the time surface memory to keep");
	double keep_fraction = get_parameter("time_surface_memory_module.keep_fraction").as_double();
	validateUnitInterval("time_surface_memory_module.keep_fraction", keep_fraction);

	time_surface_memory_module_ = std::make_unique<TimeSurfaceMemoryModule>(
		this, identifier_, topic_time_surface_memory, QOS_QUEUE, CLEAN_TIME_SPAN_SEC);
	time_surface_memory_module_->setupModule(region_of_interest_, channel, keep_fraction, time_surface_memory_timespan);
}

void FluxeemRos2Publisher::setEventToImageParams()
{
	declareParam<bool>("event_to_image_module.on_off", false, "Whether to enable the event to image module");
	bool event_to_image_on = get_parameter("event_to_image_module.on_off").as_bool();

	if (!event_to_image_on)
	{
		return;
	}

	declareParam<std::string>("event_to_image_module.topic_name", "event_frame", "The topic name for the event frame");
	std::string topic_event_to_image = get_parameter("event_to_image_module.topic_name").as_string();

	declareParam<int>("event_to_image_module.timespan", 33333,
					  "The publish interval for the event to image module, the unit is us");
	int event_to_image_timespan = get_parameter("event_to_image_module.timespan").as_int();
	validatePositiveInt("event_to_image_module.timespan", event_to_image_timespan);
	updateMinModuleTimespan(event_to_image_timespan);

	declareParam<std::string>("event_to_image_module.image_type", "time_surface", "The type of the image");
	std::string image_type = get_parameter("event_to_image_module.image_type").as_string();

	declareParam<int>("event_to_image_module.channel", 1, "The number of channel for the image");
	int channel = get_parameter("event_to_image_module.channel").as_int();

	declareParam<double>("event_to_image_module.keep_fraction", 1.0, "The fraction of the image to keep");
	double keep_fraction = get_parameter("event_to_image_module.keep_fraction").as_double();
	validateUnitInterval("event_to_image_module.keep_fraction", keep_fraction);

	event_to_image_module_ = std::make_unique<EventToImageModule>(this, identifier_, topic_event_to_image, QOS_QUEUE);
	event_to_image_module_->setupModule(image_type, region_of_interest_, channel, keep_fraction,
										event_to_image_timespan);
}

void FluxeemRos2Publisher::setParams()
{
	setCameraParams();
	if (!isRawFileMode())
	{
		setCamera();
	}
	setEventMemoryParams();
	setHistogramMemoryParams();
	setTimeSurfaceMemoryParams();
	setEventToImageParams();

	if (isRawFileMode())
	{
		if (raw_timestamp_range_known_)
		{
			resetModulesForTimestamp(raw_start_timestamp_);
		}
		configureModuleReplayMetadata(raw_file_, raw_file_hash_,
									  raw_timestamp_range_known_ ? static_cast<int64_t>(raw_start_timestamp_) : -1);
	}
	else
	{
		configureModuleReplayMetadata(recording_file_path_, "", -1);
		if (isTriggerPublishMode())
		{
			initializeTriggerWindowStart(0);
		}
	}
}
