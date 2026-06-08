#pragma once

#include <chrono>
#include <event_msgs/msg/event_memory.hpp>
#include <event_msgs/msg/histogram_memory.hpp>
#include <event_msgs/msg/time_surface_memory.hpp>
#include <fcntl.h>
#include <fluxeem_ros2_viewer/openeb_compat.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <shared_mutex>
#include <sys/mman.h>
#include <unistd.h>

inline constexpr int EVENT_MEMORY_MAGNIFICATION = 100;

class FluxeemRos2Viewer : public rclcpp::Node
{
  public:
	FluxeemRos2Viewer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	~FluxeemRos2Viewer();

  private:
	rclcpp::Subscription<event_msgs::msg::EventMemory>::SharedPtr
		event_memory_subscriber_; // EventMemory message subscriber
	rclcpp::Subscription<event_msgs::msg::HistogramMemory>::SharedPtr
		histogram_memory_subscriber_; // Histogram message subscriber
	rclcpp::Subscription<event_msgs::msg::TimeSurfaceMemory>::SharedPtr
		time_surface_memory_subscriber_;											  // TimeSurface message subscriber
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr event_frame_subscriber_; // EventFrame message subscriber
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
		camera_info_subscriber_; // CameraInfo message subscriber
	rclcpp::TimerBase::SharedPtr ui_timer_;

	uint16_t width_;				// the width of the window
	uint16_t height_;				// the height of the window
	bool camera_info_initialized_;	// a flag indicating whether camera info has been received
	bool histogram_initialized_;	// a flag indicating whether the histogram has been initialized
	bool time_surface_initialized_; // a flag indicating whether the time surface has been initialized
	bool event_frame_initialized_;	// a flag indicating whether the event frame has been initialized

	std::string histogram_window_name_;	   // the name for opencv window showing histogram
	cv::Mat histogram_;					   // the histogram of the events
	std::string time_surface_window_name_; // the name for opencv window showing time surface
	cv::Mat time_surface_;				   // the time surface of the events
	std::string event_frame_window_name_;  // the name for opencv window showing event frame
	cv::Mat event_frame_;				   // the event frame of the events
	std::string event_memory_window_name_; // the name for opencv window showing event memory
	cv::Mat event_memory_;				   // the event memory of the events
	cv::Mat event_memory_display_;

	std::string identifier_; // the identifier for the viewer, which should correspond to the publisher
	bool show_histogram_;	 // indicating whether to show histogram of the events
	bool show_time_surface_; // indicating whether to show time surface of the events
	bool show_event_frame_;	 // indicating whether to show event frame of the events
	bool show_event_memory_; // indicating whether to show event memory of the events
	int subsample_rate_;	 // subsample rate controlling the span of choosing events
	int refresh_latency_ms_;

	std::shared_mutex histogram_mutex_;	   // a mutex for histogram
	std::shared_mutex time_surface_mutex_; // a mutex for time surface
	std::shared_mutex event_frame_mutex_;  // a mutex for event frame
	std::shared_mutex event_memory_mutex_; // a mutex for event memory

	/// @brief a helper function to declare parameter in ros2
	template <typename T>
	void declareParam(const std::string &name, const T &default_value, const std::string &description);

	/// @brief set up the parameters in the ros2 node
	void setParams();

	/// @brief the callback function to deal with received event memory message
	/// @param event_memory_msg received event memory message
	void eventMemoryCallback(const event_msgs::msg::EventMemory::SharedPtr event_memory_msg);

	/// @brief the call back function to deal with received histogram memory message
	/// @param histogram_memory_msg received histogram memory message
	void histogramMemoryCallback(const event_msgs::msg::HistogramMemory::SharedPtr histogram_memory_msg);

	/// @brief the call back function to deal with received time surface memory message
	/// @param time_surface_memory_msg received time surface memory message
	void timeSurfaceMemoryCallback(const event_msgs::msg::TimeSurfaceMemory::SharedPtr time_surface_memory_msg);

	/// @brief the call back function to deal with received event frame message
	/// @param event_frame_msg received event frame message
	void eventFrameCallback(const sensor_msgs::msg::Image::SharedPtr event_frame_msg);

	/// @brief the callback function to deal with received camera info message
	/// @param camera_info received camera info message
	void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

	/// @brief initialize the data
	/// @param width the width of the window and data
	/// @param height the height of the window and data
	/// @param name the window name
	/// @param data the data that is to be initialized
	/// @param type the data type e.g. CV_8UC1
	void initWindowData(uint16_t width, uint16_t height, const std::string &name, cv::Mat &data, int type);

	/// @brief create opencv window according to the name and the camera info
	void createWindow(const std::string &window_name, unsigned int sensor_width, unsigned int sensor_height,
					  int shift_x, int shift_y);

	/// @brief calculate the histogram from shared memory
	void calculateHistogramFromMemory(const event_msgs::msg::EventMemory::SharedPtr event_memory_msg);

	/// @brief show the windows
	int processUi();
};
