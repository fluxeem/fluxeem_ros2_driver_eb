#include <fluxeem_ros2_publisher/fluxeem_ros2_publisher.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <metavision/sdk/stream/camera_stream_slicer.h>
#include <metavision/sdk/stream/file_config_hints.h>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace
{
std::string calculateFileFingerprint(const std::string &file_path)
{
	std::ifstream file(file_path, std::ios::binary);
	if (!file.is_open())
	{
		return "";
	}

	uint64_t hash = 1469598103934665603ULL;
	uint64_t file_size = 0;
	std::array<char, 1 << 20> buffer{};
	while (file)
	{
		file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
		const auto bytes_read = file.gcount();
		for (std::streamsize index = 0; index < bytes_read; index++)
		{
			hash ^= static_cast<unsigned char>(buffer[static_cast<size_t>(index)]);
			hash *= 1099511628211ULL;
		}
		file_size += static_cast<uint64_t>(bytes_read);
	}

	std::ostringstream stream;
	stream << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0') << hash << ":" << std::dec << file_size;
	return stream.str();
}

Metavision::FileConfigHints makeRawFileHints()
{
	return Metavision::FileConfigHints().real_time_playback(false).time_shift(false);
}

std::vector<TriggerEvent> convertTriggerEvents(const Metavision::TriggerBuffer &trigger_buffer)
{
	std::vector<TriggerEvent> triggers;
	triggers.reserve(trigger_buffer.size());
	for (const auto &trigger : trigger_buffer)
	{
		triggers.push_back(convertTriggerEvent(trigger));
	}
	std::sort(triggers.begin(), triggers.end(), [](const auto &lhs, const auto &rhs)
			  { return lhs.timestamp < rhs.timestamp; });
	return triggers;
}

bool getSliceTimestampOrigin(const Metavision::Slice &slice, Timestamp &origin)
{
	bool found_timestamp = false;
	auto consider_timestamp = [&](Timestamp timestamp)
	{
		if (!found_timestamp || timestamp < origin)
		{
			origin = timestamp;
		}
		found_timestamp = true;
	};

	if (slice.events && !slice.events->empty())
	{
		consider_timestamp(slice.events->front().t);
	}
	if (slice.triggers)
	{
		for (const auto &trigger : *slice.triggers)
		{
			consider_timestamp(trigger.t);
		}
	}

	return found_timestamp;
}
} // namespace

bool FluxeemRos2Publisher::openCamera()
{
	bool is_opened = false;
	if (isRawFileMode())
	{
		return openRawFile();
	}

	try
	{
		camera_ = Metavision::Camera::from_first_available();

		event_callback_id_ = camera_->cd().add_callback(
			[this](const Metavision::EventCD *begin, const Metavision::EventCD *end)
			{
				const rclcpp::Time receive_stamp = this->now();
				if (!write_enabled_ || begin == end)
				{
					return;
				}

				EventBatch events = convertEventBatch(begin, end);
				accumulateEvents(events.data(), events.data() + events.size(), receive_stamp);
			});

		if (isTriggerPublishMode())
		{
			trigger_callback_id_ = camera_->ext_trigger().add_callback(
				[this](const Metavision::EventExtTrigger *begin, const Metavision::EventExtTrigger *end)
				{
					for (const Metavision::EventExtTrigger *event = begin; event != end; event++)
					{
						const TriggerEvent trigger = convertTriggerEvent(*event);
						if (!acceptsTrigger(trigger))
						{
							continue;
						}

						handleTriggerIn(trigger, this->now());
					}
				});
		}

		camera_->add_runtime_error_callback(
			[this](const Metavision::CameraException &error)
			{
				write_enabled_ = false;
				RCLCPP_WARN(this->get_logger(), "Camera runtime error: %s", error.what());
			});

		is_opened = true;
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to open the camera: %s", e.what());
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to open the camera with an unknown error");
	}
	return is_opened;
}

bool FluxeemRos2Publisher::openRawFile()
{
	try
	{
		Metavision::Camera raw_camera = Metavision::Camera::from_file(raw_file_, makeRawFileHints());
		raw_timestamp_range_known_ = false;
		try
		{
			auto &offline_control = raw_camera.offline_streaming_control();
			if (offline_control.is_ready())
			{
				raw_start_timestamp_ = offline_control.get_seek_start_time();
				raw_end_timestamp_ = offline_control.get_seek_end_time();
				raw_timestamp_range_known_ = true;
			}
			else
			{
				RCLCPP_WARN(this->get_logger(),
							"Raw file offline controls are not ready, falling back to streaming-only playback: %s",
							raw_file_.c_str());
			}
		}
		catch (const std::exception &e)
		{
			RCLCPP_WARN(this->get_logger(),
						"Raw file offline controls are unavailable, falling back to streaming-only playback: %s",
						e.what());
		}

		if (raw_timestamp_range_known_ && raw_end_timestamp_ < raw_start_timestamp_)
		{
			RCLCPP_ERROR(this->get_logger(), "Invalid raw file timestamp range");
			return false;
		}
		if (!raw_timestamp_range_known_)
		{
			raw_start_timestamp_ = 0;
			raw_end_timestamp_ = 0;
		}

		raw_width_ = static_cast<uint16_t>(raw_camera.geometry().get_width());
		raw_height_ = static_cast<uint16_t>(raw_camera.geometry().get_height());
		raw_file_hash_ = calculateFileFingerprint(raw_file_);

		if (raw_timestamp_range_known_)
		{
			RCLCPP_INFO(this->get_logger(), "Opened raw file %s from %lld us to %lld us", raw_file_.c_str(),
						static_cast<long long>(raw_start_timestamp_), static_cast<long long>(raw_end_timestamp_));
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Opened raw file %s with streaming-only timestamp discovery",
						raw_file_.c_str());
		}
		return true;
	}
	catch (const std::exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to open raw file %s: %s", raw_file_.c_str(), e.what());
	}
	catch (...)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to open raw file %s with an unknown error", raw_file_.c_str());
	}

	return false;
}

bool FluxeemRos2Publisher::isRawFileMode() const
{
	return !raw_file_.empty();
}

void FluxeemRos2Publisher::startRawPlayback()
{
	if (!isRawFileMode() || raw_playback_running_)
	{
		return;
	}

	raw_playback_running_ = true;
	raw_playback_thread_ = std::thread(&FluxeemRos2Publisher::rawPlaybackLoop, this);
}

void FluxeemRos2Publisher::stopRawPlayback()
{
	raw_playback_running_ = false;
	raw_playback_cv_.notify_all();
	if (raw_playback_thread_.joinable() && raw_playback_thread_.get_id() != std::this_thread::get_id())
	{
		raw_playback_thread_.join();
	}
}

bool FluxeemRos2Publisher::waitForRawPlaybackTime(std::chrono::steady_clock::time_point time_point)
{
	std::unique_lock<std::mutex> lock(raw_playback_mutex_);
	return !raw_playback_cv_.wait_until(lock, time_point, [this]()
										{ return !raw_playback_running_.load() || shutdown_requested_.load(); });
}

void FluxeemRos2Publisher::rawPlaybackLoop()
{
	const int interval_us = getRawPlaybackInterval();
	RCLCPP_INFO(this->get_logger(), "Starting raw playback with %d us slices at %.3fx", interval_us,
				raw_playback_rate_);

	do
	{
		if (isTriggerPublishMode())
		{
			std::lock_guard<std::mutex> lock(raw_trigger_mutex_);
			raw_pending_triggers_.clear();
		}
		bool modules_initialized = raw_timestamp_range_known_;
		Timestamp playback_start = raw_start_timestamp_;
		Timestamp window_start = playback_start;
		auto wall_start = std::chrono::steady_clock::now();
		if (modules_initialized)
		{
			resetModulesForTimestamp(playback_start);
		}

		try
		{
			Metavision::Camera playback_camera = Metavision::Camera::from_file(raw_file_, makeRawFileHints());
			Metavision::CameraStreamSlicer slicer(
				std::move(playback_camera),
				Metavision::CameraStreamSlicer::SliceCondition::make_n_us(static_cast<Timestamp>(interval_us)), 2);

			for (const auto &slice : slicer)
			{
				if (!raw_playback_running_ || !rclcpp::ok())
				{
					break;
				}

				if (!modules_initialized)
				{
					Timestamp discovered_start = 0;
					if (!getSliceTimestampOrigin(slice, discovered_start))
					{
						continue;
					}

					raw_start_timestamp_ = discovered_start;
					playback_start = discovered_start;
					window_start = discovered_start;
					wall_start = std::chrono::steady_clock::now();
					resetModulesForTimestamp(playback_start);
					configureModuleReplayMetadata(raw_file_, raw_file_hash_, static_cast<int64_t>(playback_start));
					modules_initialized = true;
				}

				Timestamp window_end = slice.t;
				if (window_end < playback_start)
				{
					window_end = playback_start;
				}

				const double elapsed_seconds =
					static_cast<double>(window_end - playback_start) / static_cast<double>(SEC_TO_MICROSEC) /
					raw_playback_rate_;
				const auto release_time = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
														   std::chrono::duration<double>(elapsed_seconds));

				if (!waitForRawPlaybackTime(release_time))
				{
					return;
				}

				const auto event_batch = std::make_shared<EventBatch>();
				event_batch->reserve(slice.events ? slice.events->size() : 0);
				if (slice.events)
				{
					for (const auto &event : *slice.events)
					{
						event_batch->push_back(convertEvent(event));
					}
				}

				const rclcpp::Time receive_stamp = this->now();
				if (isTriggerPublishMode())
				{
					const std::vector<TriggerEvent> triggers =
						slice.triggers ? convertTriggerEvents(*slice.triggers) : std::vector<TriggerEvent>{};
					processRawTriggerModeEvents(event_batch, triggers, receive_stamp);
					publishEventMemoryUntil(window_end);
				}
				else
				{
					if (!event_batch->empty())
					{
						accumulateEvents(event_batch->data(), event_batch->data() + event_batch->size(), receive_stamp);
					}
					publishModulesUntil(window_end);
				}

				window_start = window_end;
				if (raw_timestamp_range_known_ && window_start >= raw_end_timestamp_)
				{
					break;
				}
			}
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed during raw playback: %s", e.what());
			break;
		}
	} while (raw_playback_loop_ && raw_playback_running_ && rclcpp::ok());

	raw_playback_running_ = false;
	RCLCPP_INFO(this->get_logger(), "Raw playback finished");
}

void FluxeemRos2Publisher::resetModulesForTimestamp(Timestamp timestamp_origin)
{
	std::lock_guard<std::mutex> lock(module_mutex_);
	forEachWindowedModule(
		[timestamp_origin](auto &module)
		{
			module.reset();
			module.setTimestampOrigin(timestamp_origin);
		});

	if (isTriggerPublishMode())
	{
		initializeTriggerWindowStart(timestamp_origin);
	}
}

void FluxeemRos2Publisher::publishModulesUntil(Timestamp timestamp)
{
	std::lock_guard<std::mutex> lock(module_mutex_);
	if (!write_enabled_)
	{
		return;
	}

	try
	{
		forEachWindowedModule([timestamp](auto &module) { module.publishUntil(timestamp); });
	}
	catch (const std::exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to publish raw playback window, the error is %s", e.what());
	}
}

void FluxeemRos2Publisher::publishEventMemoryUntil(Timestamp timestamp)
{
	std::lock_guard<std::mutex> lock(module_mutex_);
	if (!write_enabled_)
	{
		return;
	}

	try
	{
		if (event_memory_module_)
		{
			event_memory_module_->publishUntil(timestamp);
		}
	}
	catch (const std::exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to publish event memory window, the error is %s", e.what());
	}
}

bool FluxeemRos2Publisher::isTriggerPublishMode() const
{
	return trigger_publish_mode_;
}

bool FluxeemRos2Publisher::acceptsTrigger(const TriggerEvent &trigger) const
{
	return isTriggerPublishMode() && trigger.polarity == trigger_polarity_;
}

void FluxeemRos2Publisher::initializeTriggerWindowStart(Timestamp timestamp_origin)
{
	trigger_window_start_timestamp_ = timestamp_origin;
	forEachTriggerWindowModule([timestamp_origin](auto &module) { module.setTriggerWindowStart(timestamp_origin); });
}

void FluxeemRos2Publisher::handleTriggerIn(const TriggerEvent &trigger, const rclcpp::Time &receive_stamp)
{
	std::lock_guard<std::mutex> lock(module_mutex_);
	if (!write_enabled_ || !acceptsTrigger(trigger))
	{
		return;
	}

	try
	{
		if (event_memory_module_)
		{
			event_memory_module_->publishTriggerWindow(trigger, receive_stamp);
		}
		forEachTriggerWindowModule(
			[&](auto &module) { module.publishTriggerWindow(trigger_window_start_timestamp_, trigger, receive_stamp); });

		trigger_window_start_timestamp_ = trigger.timestamp;
	}
	catch (const std::exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to publish trigger window, the error is %s", e.what());
	}
}

std::vector<TriggerEvent> FluxeemRos2Publisher::takeRawTriggersUntil(Timestamp timestamp)
{
	std::vector<TriggerEvent> triggers;
	std::lock_guard<std::mutex> lock(raw_trigger_mutex_);
	while (!raw_pending_triggers_.empty() && raw_pending_triggers_.front().timestamp <= timestamp)
	{
		triggers.push_back(raw_pending_triggers_.front());
		raw_pending_triggers_.pop_front();
	}

	std::sort(triggers.begin(), triggers.end(), [](const auto &lhs, const auto &rhs)
			  { return lhs.timestamp < rhs.timestamp; });
	return triggers;
}

void FluxeemRos2Publisher::processRawTriggerModeEvents(const std::shared_ptr<EventBatch> &event_batch,
													   const std::vector<TriggerEvent> &triggers,
													   const rclcpp::Time &receive_stamp)
{
	const Event *current = event_batch && !event_batch->empty() ? event_batch->data() : nullptr;
	const Event *end = event_batch && !event_batch->empty() ? event_batch->data() + event_batch->size() : nullptr;

	for (const auto &trigger : triggers)
	{
		const Event *trigger_end = current;
		if (current != nullptr)
		{
			trigger_end = std::upper_bound(current, end, trigger.timestamp,
										   [](const Timestamp &timestamp, const Event &event)
										   { return timestamp < event.timestamp; });
			if (trigger_end != current)
			{
				accumulateEvents(current, trigger_end, receive_stamp);
			}
			current = trigger_end;
		}

		handleTriggerIn(trigger, receive_stamp);
	}

	if (current != nullptr && current != end)
	{
		accumulateEvents(current, end, receive_stamp);
	}
}

void FluxeemRos2Publisher::updateMinModuleTimespan(int timespan)
{
	if (timespan <= 0)
	{
		return;
	}

	if (min_module_timespan_ == 0 || timespan < min_module_timespan_)
	{
		min_module_timespan_ = timespan;
	}
}

int FluxeemRos2Publisher::getRawPlaybackInterval() const
{
	if (raw_playback_interval_us_ > 0)
	{
		return raw_playback_interval_us_;
	}
	if (min_module_timespan_ > 0)
	{
		return min_module_timespan_;
	}
	return 33333;
}

void FluxeemRos2Publisher::configureModuleReplayMetadata(const std::string &raw_file_path,
														 const std::string &raw_file_hash,
														 int64_t raw_file_start_timestamp)
{
	forEachSharedMemoryModule(
		[&](auto &module) { module.setReplaySourceMetadata(raw_file_path, raw_file_hash, raw_file_start_timestamp); });
}

template <typename InputIt>
void FluxeemRos2Publisher::accumulateEvents(InputIt iter_start, InputIt iter_end, const rclcpp::Time &receive_stamp)
{
	if (!write_enabled_)
	{
		RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100,
							  "Banned from writing and current size is: %lu", iter_end - iter_start);
		return;
	}

	if (iter_start == iter_end)
	{
		return;
	}

	std::lock_guard<std::mutex> lock(module_mutex_);

	if (!write_enabled_)
	{
		return;
	}

	try
	{
		if (event_memory_module_)
		{
			event_memory_module_->processEvents(iter_start, iter_end, receive_stamp);
		}
		if (isTriggerPublishMode())
		{
			if (histogram_memory_module_)
			{
				histogram_memory_module_->processEventsWithoutWindow(iter_start, iter_end, receive_stamp);
			}
			if (time_surface_memory_module_)
			{
				time_surface_memory_module_->processEventsWithoutWindow(iter_start, iter_end, receive_stamp);
			}
			if (event_to_image_module_)
			{
				event_to_image_module_->processEventsWithoutWindow(iter_start, iter_end, receive_stamp);
			}
			return;
		}
		if (histogram_memory_module_)
		{
			histogram_memory_module_->processEvents(iter_start, iter_end, receive_stamp);
		}
		if (time_surface_memory_module_)
		{
			time_surface_memory_module_->processEvents(iter_start, iter_end, receive_stamp);
		}
		if (event_to_image_module_)
		{
			event_to_image_module_->processEvents(iter_start, iter_end, receive_stamp);
		}
	}
	catch (std::exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to write to shared memory, the error is %s", e.what());
	}
}
