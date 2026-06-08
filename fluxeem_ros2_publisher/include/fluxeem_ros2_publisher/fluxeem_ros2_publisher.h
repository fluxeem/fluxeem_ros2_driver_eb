#pragma once

#include <camera_interfaces/srv/camera_info.hpp>
#include <camera_interfaces/srv/record.hpp>
#include <camera_interfaces/srv/set_biases.hpp>
#include <camera_interfaces/srv/set_erc.hpp>
#include <camera_interfaces/srv/set_trail_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "camera_settings.hpp"
#include "modules.hpp"
#include "openeb_compat.hpp"
#include "publisher_constant.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fcntl.h>
#include <metavision/sdk/stream/camera.h>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <unordered_set>
#include <utility>
#include <vector>

class FluxeemRos2Publisher : public rclcpp::Node
{
  public:
	FluxeemRos2Publisher(const rclcpp::NodeOptions &options);
	~FluxeemRos2Publisher();

	/// @brief stop the camera and release all shared memory owned by this node
	void cleanShutdown();

  private:
	// Set up the parameters
	void setGeneralParams();
	void setPublishStrategyParams();
	void setCameraParams();
	void setEventMemoryParams();
	void setHistogramMemoryParams();
	void setTimeSurfaceMemoryParams();
	void setEventToImageParams();

	/// @brief publish camera_info
	void publishCameraInfo();

	/// @brief set up the parameters in the ros2 node
	void setParams();

	/// @brief open the camera
	bool openCamera();

	/// @brief reopen the camera, will be called when the connection is not secured
	void reopenCamera();

	bool openRawFile();
	bool isRawFileMode() const;
	void startRawPlayback();
	void stopRawPlayback();
	void rawPlaybackLoop();
	bool waitForRawPlaybackTime(std::chrono::steady_clock::time_point time_point);
	void resetModulesForTimestamp(Timestamp timestamp_origin);
	void publishModulesUntil(Timestamp timestamp);
	void publishEventMemoryUntil(Timestamp timestamp);
	void updateMinModuleTimespan(int timespan);
	int getRawPlaybackInterval() const;
	void configureModuleReplayMetadata(const std::string &raw_file_path, const std::string &raw_file_hash,
									   int64_t raw_file_start_timestamp);
	bool isTriggerPublishMode() const;
	bool acceptsTrigger(const TriggerEvent &trigger) const;
	void initializeTriggerWindowStart(Timestamp timestamp_origin);
	void handleTriggerIn(const TriggerEvent &trigger, const rclcpp::Time &receive_stamp);
	void processRawTriggerModeEvents(const std::shared_ptr<EventBatch> &event_batch,
									 const std::vector<TriggerEvent> &triggers,
									 const rclcpp::Time &receive_stamp);
	std::vector<TriggerEvent> takeRawTriggersUntil(Timestamp timestamp);
	template <typename Func>
	void forEachWindowedModule(Func &&func);
	template <typename Func>
	void forEachSharedMemoryModule(Func &&func);
	template <typename Func>
	void forEachTriggerWindowModule(Func &&func);

	/// @brief accumulate the events into the event array
	template <typename InputIt>
	void accumulateEvents(InputIt iter_start, InputIt iter_end, const rclcpp::Time &receive_stamp);

	/// @brief a helper function to declare parameter in ros2
	template <typename T>
	void declareParam(const std::string &name, const T &default_value, const std::string &description);

	void cameraInfoServiceCallback(const camera_interfaces::srv::CameraInfo::Request::SharedPtr request,
								   camera_interfaces::srv::CameraInfo::Response::SharedPtr response);

	void setBiasesServiceCallback(const camera_interfaces::srv::SetBiases::Request::SharedPtr request,
								  camera_interfaces::srv::SetBiases::Response::SharedPtr response);

	void setEventRateControlServiceCallback(const camera_interfaces::srv::SetErc::Request::SharedPtr request,
											camera_interfaces::srv::SetErc::Response::SharedPtr response);

	void setTrailFilterServiceCallback(const camera_interfaces::srv::SetTrailFilter::Request::SharedPtr request,
									   camera_interfaces::srv::SetTrailFilter::Response::SharedPtr response);

	void recordServiceCallback(const camera_interfaces::srv::Record::Request::SharedPtr request,
							   camera_interfaces::srv::Record::Response::SharedPtr response);

	/// @brief set the biases
	/// @return true if success, false otherwise
	bool setBiases();

	/// @brief get the biases
	/// @return true if success, false otherwise
	bool getBiasesInfo(int &bias_diff, int &bias_diff_on, int &bias_diff_off, int &bias_fo, int &bias_hpf, int &bias_refr);

	/// @brief set event rate control
	/// @return true if success, false otherwise
	bool setEventRateControl();

	/// @brief get event rate control
	/// @return true if success, false otherwise
	bool getEventRateControlInfo(bool &enabled, int &rate);

	/// @brief set the trail filter
	/// @return true if success, false otherwise
	bool setTrailFilter();

	/// @brief get the trail filter
	/// @return true if success, false otherwise
	bool getTrailFilterInfo(bool &enabled, std::string &filter_type, int &threshold);

	/// @brief set the camera including its biases, event rate control and trail filter
	void setCamera();

  private:
	sensor_msgs::msg::CameraInfo camera_info_;										   // camera info message
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_; // camera info publisher pointer
	rclcpp::TimerBase::SharedPtr publish_timer_;

	// services for camera
	rclcpp::Service<camera_interfaces::srv::CameraInfo>::SharedPtr camera_info_service_;
	rclcpp::Service<camera_interfaces::srv::SetBiases>::SharedPtr set_biases_service_;
	rclcpp::Service<camera_interfaces::srv::SetErc>::SharedPtr set_event_rate_control_service_;
	rclcpp::Service<camera_interfaces::srv::SetTrailFilter>::SharedPtr set_trail_filter_service_;
	rclcpp::Service<camera_interfaces::srv::Record>::SharedPtr record_service_;
	std::string raw_file_; // the path of the .raw file if the camera will be read from it
	std::string raw_file_hash_;
	std::string recording_file_path_;
	std::string identifier_; // the identifier will determine the final topic name and frame_id of the publication

	CameraSettings camera_settings_;

	// RegionOfInterest related
	double width_fraction_;
	double height_fraction_;
	RegionOfInterest region_of_interest_;

	// camera related
	std::optional<Metavision::Camera> camera_; // camera instance
	std::atomic<bool> write_enabled_; // if the camera is under reopening, the memory writing is not allowed
	std::atomic<bool> shutdown_requested_;
	std::atomic<bool> raw_playback_running_;
	std::mutex module_mutex_;
	std::mutex raw_trigger_mutex_;
	std::mutex raw_playback_mutex_;
	std::condition_variable raw_playback_cv_;
	std::thread raw_playback_thread_;
	Timestamp raw_start_timestamp_;
	Timestamp raw_end_timestamp_;
	bool raw_timestamp_range_known_;
	uint16_t raw_width_;
	uint16_t raw_height_;
	int raw_playback_interval_us_;
	double raw_playback_rate_;
	bool raw_playback_loop_;
	int min_module_timespan_;
	uint32_t event_callback_id_; // callback function id
	uint32_t trigger_callback_id_;
	bool trigger_publish_mode_;
	int trigger_polarity_;
	Timestamp trigger_window_start_timestamp_;
	std::deque<TriggerEvent> raw_pending_triggers_;

	std::unique_ptr<EventMemoryModule> event_memory_module_;
	std::unique_ptr<HistogramMemoryModule> histogram_memory_module_;
	std::unique_ptr<TimeSurfaceMemoryModule> time_surface_memory_module_;
	std::unique_ptr<EventToImageModule> event_to_image_module_;
};

template <typename Func>
void FluxeemRos2Publisher::forEachWindowedModule(Func &&func)
{
	if (event_memory_module_)
	{
		func(*event_memory_module_);
	}
	if (histogram_memory_module_)
	{
		func(*histogram_memory_module_);
	}
	if (time_surface_memory_module_)
	{
		func(*time_surface_memory_module_);
	}
	if (event_to_image_module_)
	{
		func(*event_to_image_module_);
	}
}

template <typename Func>
void FluxeemRos2Publisher::forEachSharedMemoryModule(Func &&func)
{
	if (event_memory_module_)
	{
		func(*event_memory_module_);
	}
	if (histogram_memory_module_)
	{
		func(*histogram_memory_module_);
	}
	if (time_surface_memory_module_)
	{
		func(*time_surface_memory_module_);
	}
}

template <typename Func>
void FluxeemRos2Publisher::forEachTriggerWindowModule(Func &&func)
{
	if (histogram_memory_module_)
	{
		func(*histogram_memory_module_);
	}
	if (time_surface_memory_module_)
	{
		func(*time_surface_memory_module_);
	}
	if (event_to_image_module_)
	{
		func(*event_to_image_module_);
	}
}
