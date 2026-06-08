#include <fluxeem_ros2_publisher/fluxeem_ros2_publisher.h>

#include <array>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <functional>
#include <stdexcept>

namespace
{
std::array<int, 2> g_signal_pipe{{-1, -1}};
std::atomic<bool> g_signal_bridge_installed{false};
volatile std::sig_atomic_t g_shutdown_signal = 0;

void signalRelayHandler(int signal_number)
{
	if (g_shutdown_signal != 0)
	{
		_exit(128 + signal_number);
	}

	g_shutdown_signal = signal_number;
	if (g_signal_pipe[1] == -1)
	{
		return;
	}

	const unsigned char signal_byte = static_cast<unsigned char>(signal_number);
	const ssize_t ignored = write(g_signal_pipe[1], &signal_byte, sizeof(signal_byte));
	(void)ignored;
}

bool installSignalBridge(const std::weak_ptr<FluxeemRos2Publisher> &weak_node)
{
	bool expected = false;
	if (!g_signal_bridge_installed.compare_exchange_strong(expected, true))
	{
		return true;
	}

	if (pipe(g_signal_pipe.data()) == -1)
	{
		g_signal_bridge_installed = false;
		return false;
	}

	struct sigaction action{};
	action.sa_handler = signalRelayHandler;
	sigemptyset(&action.sa_mask);
	action.sa_flags = SA_RESTART;

	if (sigaction(SIGINT, &action, nullptr) == -1 || sigaction(SIGTERM, &action, nullptr) == -1)
	{
		close(g_signal_pipe[0]);
		close(g_signal_pipe[1]);
		g_signal_pipe = {{-1, -1}};
		g_signal_bridge_installed = false;
		return false;
	}

	std::thread(
		[weak_node]()
		{
			unsigned char signal_byte = 0;
			const ssize_t bytes_read = read(g_signal_pipe[0], &signal_byte, sizeof(signal_byte));
			if (bytes_read <= 0)
			{
				return;
			}

			if (auto node = weak_node.lock())
			{
				node->cleanShutdown();
			}

			if (rclcpp::ok())
			{
				rclcpp::shutdown();
			}
		})
		.detach();

	return true;
}
} // namespace


FluxeemRos2Publisher::FluxeemRos2Publisher(const rclcpp::NodeOptions &options)
	: Node("fluxeem_ros2_publisher_node", options), write_enabled_(true), shutdown_requested_(false),
	  raw_playback_running_(false), raw_start_timestamp_(0), raw_end_timestamp_(0), raw_timestamp_range_known_(false),
	  raw_width_(0), raw_height_(0), raw_playback_interval_us_(0), raw_playback_rate_(1.0), raw_playback_loop_(false),
	  min_module_timespan_(0), event_callback_id_(0), trigger_callback_id_(0), trigger_publish_mode_(false),
	  trigger_polarity_(1), trigger_window_start_timestamp_(0)
{
	setGeneralParams();

	if (isRawFileMode())
	{
		if (!openCamera())
		{
			throw std::runtime_error("Failed to open the raw file");
		}
	}
	else
	{
		while (!openCamera())
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
			RCLCPP_INFO(this->get_logger(), "Trying to open the camera");
		}
	}

	setParams();

	const std::string topic_camera_info = identifier_ + "/camera_info";

	camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
		topic_camera_info, rclcpp::QoS(rclcpp::KeepLast(QOS_QUEUE)).best_effort().durability_volatile());

	camera_info_service_ = this->create_service<camera_interfaces::srv::CameraInfo>(
		identifier_ + "/camera_info_srv", std::bind(&FluxeemRos2Publisher::cameraInfoServiceCallback, this,
													std::placeholders::_1, std::placeholders::_2));

	set_biases_service_ = this->create_service<camera_interfaces::srv::SetBiases>(
		identifier_ + "/set_biases_srv",
		std::bind(&FluxeemRos2Publisher::setBiasesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

	set_event_rate_control_service_ = this->create_service<camera_interfaces::srv::SetErc>(
		identifier_ + "/set_erc_srv", std::bind(&FluxeemRos2Publisher::setEventRateControlServiceCallback, this,
												std::placeholders::_1, std::placeholders::_2));

	set_trail_filter_service_ = this->create_service<camera_interfaces::srv::SetTrailFilter>(
		identifier_ + "/set_trail_filter_srv", std::bind(&FluxeemRos2Publisher::setTrailFilterServiceCallback, this,
														 std::placeholders::_1, std::placeholders::_2));

	record_service_ = this->create_service<camera_interfaces::srv::Record>(
		identifier_ + "/record_srv",
		std::bind(&FluxeemRos2Publisher::recordServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

	if (isRawFileMode())
	{
		startRawPlayback();
	}
	else
	{
		if (!camera_->start())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to start the camera");
			throw std::runtime_error("Failed to start the camera");
		}
	}

	publish_timer_ =
		create_wall_timer(std::chrono::milliseconds(33), std::bind(&FluxeemRos2Publisher::publishCameraInfo, this));
}

FluxeemRos2Publisher::~FluxeemRos2Publisher()
{
	cleanShutdown();
}

void FluxeemRos2Publisher::cleanShutdown()
{
	bool expected = false;
	if (!shutdown_requested_.compare_exchange_strong(expected, true))
	{
		return;
	}

	write_enabled_ = false;
	stopRawPlayback();

	if (publish_timer_)
	{
		publish_timer_->cancel();
	}

	{
		std::lock_guard<std::mutex> lock(module_mutex_);
		forEachSharedMemoryModule([](auto &module) { module.cleanSharedMemory(true); });

		event_to_image_module_.reset();
		time_surface_memory_module_.reset();
		histogram_memory_module_.reset();
		event_memory_module_.reset();
	}

	if (camera_)
	{
		try
		{
			camera_->stop();
		}
		catch (...)
		{
			RCLCPP_WARN(this->get_logger(), "Failed to stop the camera during clean shutdown");
		}
	}

	if (camera_)
	{
		camera_.reset();
	}
}

void FluxeemRos2Publisher::publishCameraInfo()
{
	if (shutdown_requested_)
	{
		return;
	}

	if (!isRawFileMode() && camera_ && !camera_->is_running())
	{
		write_enabled_ = false;
		try
		{
			reopenCamera();
		}
		catch (...)
		{
			RCLCPP_WARN(this->get_logger(), "Reopen failed, try again");
			std::this_thread::sleep_for(std::chrono::seconds(1));
			return;
		}
	}

	if (camera_info_publisher_->get_subscription_count() > 0)
	{
		camera_info_.header.stamp = this->now();
		camera_info_publisher_->publish(camera_info_);
	}
}

void FluxeemRos2Publisher::reopenCamera()
{
	if (isRawFileMode())
	{
		return;
	}

	camera_->stop();
	camera_.reset();
	while (!openCamera())
	{
		RCLCPP_INFO(this->get_logger(), "Trying to open the camera");
		std::this_thread::sleep_for(std::chrono::milliseconds(REST_TIME_SPAN));
	}

	setCamera();

	{
		std::lock_guard<std::mutex> lock(module_mutex_);

		forEachWindowedModule([](auto &module) { module.reset(); });

		if (isTriggerPublishMode())
		{
			initializeTriggerWindowStart(0);
		}
	}

	if (!camera_->start())
	{
		throw std::runtime_error("Failed to restart the camera");
	}
	write_enabled_ = true;
}


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);
	auto node = std::make_shared<FluxeemRos2Publisher>(options);
	std::weak_ptr<FluxeemRos2Publisher> weak_node = node;

	if (!installSignalBridge(weak_node))
	{
		RCLCPP_WARN(node->get_logger(), "Failed to install the SIGINT cleanup bridge");
	}

	rclcpp::on_shutdown(
		[weak_node]()
		{
			if (auto node = weak_node.lock())
			{
				node->cleanShutdown();
			}
		});

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);

	executor.spin();

	node->cleanShutdown();
	node.reset();
	if (rclcpp::ok())
	{
		rclcpp::shutdown();
	}
	return 0;
}
