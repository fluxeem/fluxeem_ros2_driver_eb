#include <fluxeem_ros2_viewer/fluxeem_ros2_viewer.h>

#include <algorithm>
#include <cstring>

FluxeemRos2Viewer::FluxeemRos2Viewer(const rclcpp::NodeOptions &options)
	: Node("fluxeem_ros2_viewer_node", options), width_(0), height_(0), camera_info_initialized_(false),
	  histogram_initialized_(false), time_surface_initialized_(false), event_frame_initialized_(false),
	  histogram_window_name_("event histogram"), time_surface_window_name_("event time surface"),
	  event_frame_window_name_("event frame"), event_memory_window_name_("event frame computed from events")
{
	// set up necessary parameters
	setParams();

	// set up the publishers
	const std::string topic_camera_info = identifier_ + "/camera_info";
	const std::string topic_event_memory = identifier_ + "/event_memory";
	const std::string topic_histogram_memory = identifier_ + "/histogram_memory";
	const std::string topic_time_surface_memory = identifier_ + "/time_surface_memory";
	const std::string topic_event_frame = identifier_ + "/event_frame";

	camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
		topic_camera_info, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
		std::bind(&FluxeemRos2Viewer::cameraInfoCallback, this, std::placeholders::_1));

	ui_timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&FluxeemRos2Viewer::processUi, this));

	event_memory_subscriber_ = create_subscription<event_msgs::msg::EventMemory>(
		topic_event_memory, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
		std::bind(&FluxeemRos2Viewer::eventMemoryCallback, this, std::placeholders::_1));
	if (show_histogram_)
	{
		histogram_memory_subscriber_ = create_subscription<event_msgs::msg::HistogramMemory>(
			topic_histogram_memory, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
			std::bind(&FluxeemRos2Viewer::histogramMemoryCallback, this, std::placeholders::_1));
	}
	if (show_time_surface_)
	{
		time_surface_memory_subscriber_ = create_subscription<event_msgs::msg::TimeSurfaceMemory>(
			topic_time_surface_memory, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
			std::bind(&FluxeemRos2Viewer::timeSurfaceMemoryCallback, this, std::placeholders::_1));
	}
	if (show_event_frame_)
	{
		event_frame_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
			topic_event_frame, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
			std::bind(&FluxeemRos2Viewer::eventFrameCallback, this, std::placeholders::_1));
	}
}

FluxeemRos2Viewer::~FluxeemRos2Viewer()
{
	cv::destroyAllWindows();
}

template <class T>
void FluxeemRos2Viewer::declareParam(const std::string &name, const T &default_value, const std::string &description)
{
	rcl_interfaces::msg::ParameterDescriptor desc;
	desc.description = description;
	this->declare_parameter(name, default_value, desc);
}

// set up all the parameters
void FluxeemRos2Viewer::setParams()
{
	declareParam<std::string>("identifier", "fluxeem",
							  "A string as an identifier, which helps distinguish from others");
	identifier_ = get_parameter("identifier").as_string();

	declareParam<bool>("show_histogram", false, "A flag indicating whether to show time histogram of the events");
	show_histogram_ = get_parameter("show_histogram").as_bool();

	declareParam<bool>("show_time_surface", false, "A flag indicating whether to show time surface of the events");
	show_time_surface_ = get_parameter("show_time_surface").as_bool();

	declareParam<bool>("show_event_frame", false, "A flag indicating whether to show event frame of the events");
	show_event_frame_ = get_parameter("show_event_frame").as_bool();

	declareParam<bool>("show_event_memory", false, "A flag indicating whether to show event memory of the events");
	show_event_memory_ = get_parameter("show_event_memory").as_bool();

	declareParam<int>("subsample_rate", 1, "A number that controls the subsample rate when we choose copy events");
	subsample_rate_ = get_parameter("subsample_rate").as_int();

	declareParam<int>("refresh_latency", 1, "milliseconds");
	refresh_latency_ms_ = get_parameter("refresh_latency").as_int();
}

void FluxeemRos2Viewer::eventMemoryCallback(const event_msgs::msg::EventMemory::SharedPtr event_memory_msg)
{
	std::unique_lock<std::shared_mutex> lock(event_memory_mutex_);
	if (!camera_info_initialized_)
		return;

	calculateHistogramFromMemory(event_memory_msg);
}

void FluxeemRos2Viewer::histogramMemoryCallback(const event_msgs::msg::HistogramMemory::SharedPtr histogram_memory_msg)
{
	std::unique_lock<std::shared_mutex> lock(histogram_mutex_);

	uint16_t width = static_cast<uint16_t>(histogram_memory_msg->width);
	uint16_t height = static_cast<uint16_t>(histogram_memory_msg->height);
	uint8_t channel = histogram_memory_msg->channel;

	if (!histogram_initialized_)
	{
		initWindowData(width, height, histogram_window_name_, histogram_, CV_8UC(channel));
		histogram_initialized_ = true;
	}

	auto memory_size = sizeof(uint8_t) * width * height * channel;

	std::string memory_identifier = histogram_memory_msg->identifier;

	int shared_memory_fd = shm_open(memory_identifier.c_str(), O_RDONLY, S_IRUSR | S_IRGRP | S_IROTH);
	if (shared_memory_fd == -1)
	{
		RCLCPP_WARN(this->get_logger(), "The shared memory is not open for histogram reader");
		return;
	}

	void *shared_memory_ptr = mmap(0, memory_size, PROT_READ, MAP_SHARED, shared_memory_fd, 0);
	if (shared_memory_ptr == MAP_FAILED)
	{
		RCLCPP_WARN(this->get_logger(), "The shared memory pointer is not mapped successfully for histogram reader");
		return;
	}
	char *histogram_data = static_cast<char *>(shared_memory_ptr);

	memcpy(histogram_.data, histogram_data, width * height * channel);

	for (size_t pixel_index = 0; pixel_index < width * height * channel; pixel_index++)
	{
		if (histogram_.data[pixel_index])
		{
			histogram_.data[pixel_index] = static_cast<uint8_t>(
				std::min(255, static_cast<int>(histogram_.data[pixel_index]) + EVENT_MEMORY_MAGNIFICATION));
		}
	}

	if (munmap(shared_memory_ptr, memory_size) == -1)
	{
		RCLCPP_WARN(this->get_logger(), "Unmap the data pointer has failed.");
	}
	close(shared_memory_fd);
}

void FluxeemRos2Viewer::timeSurfaceMemoryCallback(
	const event_msgs::msg::TimeSurfaceMemory::SharedPtr time_surface_memory_msg)
{
	std::unique_lock<std::shared_mutex> lock(time_surface_mutex_);

	uint16_t width = static_cast<uint16_t>(time_surface_memory_msg->width);
	uint16_t height = static_cast<uint16_t>(time_surface_memory_msg->height);
	uint8_t channel = time_surface_memory_msg->channel;

	if (!time_surface_initialized_)
	{
		initWindowData(width, height, time_surface_window_name_, time_surface_, CV_8UC(channel));
		time_surface_initialized_ = true;
	}

	auto memory_size = sizeof(uint8_t) * width * height * channel;

	std::string memory_identifier = time_surface_memory_msg->identifier;

	int shared_memory_fd = shm_open(memory_identifier.c_str(), O_RDONLY, S_IRUSR | S_IRGRP | S_IROTH);
	if (shared_memory_fd == -1)
	{
		RCLCPP_WARN(this->get_logger(), "The shared memory is not open for time surface reader");
		return;
	}

	void *shared_memory_ptr = mmap(0, memory_size, PROT_READ, MAP_SHARED, shared_memory_fd, 0);
	if (shared_memory_ptr == MAP_FAILED)
	{
		RCLCPP_WARN(this->get_logger(), "The shared memory pointer is not mapped successfully for time surface reader");
		return;
	}
	char *time_surface_data = static_cast<char *>(shared_memory_ptr);

	memcpy(time_surface_.data, time_surface_data, width * height * channel);

	if (munmap(shared_memory_ptr, memory_size) == -1)
	{
		RCLCPP_WARN(this->get_logger(), "Unmap the data pointer has failed.");
	}
	close(shared_memory_fd);
}

void FluxeemRos2Viewer::eventFrameCallback(const sensor_msgs::msg::Image::SharedPtr event_frame_msg)
{
	std::unique_lock<std::shared_mutex> lock(event_frame_mutex_);

	if (!event_frame_initialized_)
	{
		int channel = event_frame_msg->step / event_frame_msg->width;
		initWindowData(event_frame_msg->width, event_frame_msg->height, event_frame_window_name_, event_frame_,
					   CV_8UC(channel));
		event_frame_initialized_ = true;
	}

	memcpy(event_frame_.data, event_frame_msg->data.data(), event_frame_msg->data.size());
}

void FluxeemRos2Viewer::calculateHistogramFromMemory(const event_msgs::msg::EventMemory::SharedPtr event_memory_msg)
{
	std::string memory_identifier = event_memory_msg->identifier;
	uint64_t length = event_memory_msg->length;
	uint64_t memory_size = sizeof(Event) * length;

	event_memory_ = cv::Mat(height_ / event_memory_msg->spatial_sample_rate,
							width_ / event_memory_msg->spatial_sample_rate, CV_8UC1, cv::Scalar(128));

	if (length == 0)
		return; // skip if no events are available

	RCLCPP_DEBUG(this->get_logger(), "Received memory size is %lu", memory_size);

	int shared_memory_fd = shm_open(memory_identifier.c_str(), O_RDONLY, S_IRUSR | S_IRGRP | S_IROTH);
	if (shared_memory_fd == -1)
	{
		RCLCPP_FATAL(this->get_logger(), "The shared memory is not open");
	}

	void *shared_memory_ptr = mmap(0, memory_size, PROT_READ, MAP_SHARED, shared_memory_fd, 0);
	if (shared_memory_ptr == MAP_FAILED)
	{
		RCLCPP_FATAL(this->get_logger(), "The shared memory pointer is not mapped successfully");
	}
	Event *events = static_cast<Event *>(shared_memory_ptr);

	for (size_t event_index = 0; event_index < length; ++event_index)
	{
		if (events[event_index].polarity)
		{
			event_memory_.at<uint8_t>(events[event_index].y, events[event_index].x) = 255;
		}
		else
		{
			event_memory_.at<uint8_t>(events[event_index].y, events[event_index].x) = 0;
		}
	}

	std::swap(event_memory_, event_memory_display_);

	if (munmap(shared_memory_ptr, memory_size) == -1)
	{
		RCLCPP_WARN(this->get_logger(), "Unmap the data pointer has failed.");
	}
	close(shared_memory_fd);
}

void FluxeemRos2Viewer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
	RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Received camera info, initialization status %s",
						  camera_info_initialized_ ? "true" : "false");
	if (camera_info_initialized_)
		return;

	width_ = camera_info_msg->width;
	height_ = camera_info_msg->height;

	if (width_ != 0 && height_ != 0)
	{
		camera_info_initialized_ = true;
	}
}

void FluxeemRos2Viewer::initWindowData(uint16_t width, uint16_t height, const std::string &name, cv::Mat &data,
									   int type)
{
	data = cv::Mat(height, width, type);
	data.setTo(cv::Scalar(0));

	createWindow(name, width, height, 0, 0);
}

void FluxeemRos2Viewer::createWindow(const std::string &window_name, unsigned int sensor_width,
									 unsigned int sensor_height, int shift_x, int shift_y)
{
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	cv::resizeWindow(window_name, sensor_width, sensor_height);
	// move needs to be after resize on apple, otherwise the window stacks
	cv::moveWindow(window_name, shift_x, shift_y);
}

int FluxeemRos2Viewer::processUi()
{
	if (show_histogram_ && histogram_initialized_)
	{
		std::shared_lock<std::shared_mutex> lock(histogram_mutex_);
		cv::imshow(histogram_window_name_, histogram_);
	}

	if (show_time_surface_ && time_surface_initialized_)
	{
		std::shared_lock<std::shared_mutex> lock(time_surface_mutex_);
		cv::imshow(time_surface_window_name_, time_surface_);
	}

	if (show_event_frame_ && event_frame_initialized_)
	{
		std::shared_lock<std::shared_mutex> lock(event_frame_mutex_);
		cv::imshow(event_frame_window_name_, event_frame_);
	}

	if (show_event_memory_ && camera_info_initialized_ && !event_memory_display_.empty())
	{
		std::shared_lock<std::shared_mutex> lock(event_memory_mutex_);
		cv::imshow(event_memory_window_name_, event_memory_display_);
	}

	int key = cv::waitKey(refresh_latency_ms_);

	return key;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);
	auto node = std::make_shared<FluxeemRos2Viewer>(options);
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);

	executor.spin();

	rclcpp::shutdown();

	return 0;
}
