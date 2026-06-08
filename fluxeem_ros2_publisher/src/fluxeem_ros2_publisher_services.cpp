#include <fluxeem_ros2_publisher/fluxeem_ros2_publisher.h>

#include <exception>

void FluxeemRos2Publisher::cameraInfoServiceCallback(
	const camera_interfaces::srv::CameraInfo::Request::SharedPtr request,
	camera_interfaces::srv::CameraInfo::Response::SharedPtr response)
{
	(void)request;
	response->width = camera_info_.width;
	response->height = camera_info_.height;

	getTrailFilterInfo(response->trail_filter_enabled, response->trail_filter_type, response->trail_filter_threshold);
	getEventRateControlInfo(response->erc_enabled, response->erc_rate);
	getBiasesInfo(response->bias_diff, response->bias_diff_on, response->bias_diff_off, response->bias_fo, response->bias_hpf, response->bias_refr);
}

void FluxeemRos2Publisher::setBiasesServiceCallback(const camera_interfaces::srv::SetBiases::Request::SharedPtr request,
													camera_interfaces::srv::SetBiases::Response::SharedPtr response)
{
	if (isRawFileMode())
	{
		RCLCPP_WARN(this->get_logger(), "Bias service is not available during raw file playback");
		response->success = false;
		return;
	}

	camera_settings_.biases_.setParams(request->bias_diff, request->bias_diff_on, request->bias_diff_off,
									   request->bias_fo, request->bias_hpf, request->bias_refr);
	response->success = setBiases();
}

void FluxeemRos2Publisher::setEventRateControlServiceCallback(
	const camera_interfaces::srv::SetErc::Request::SharedPtr request,
	camera_interfaces::srv::SetErc::Response::SharedPtr response)
{
	if (isRawFileMode())
	{
		RCLCPP_WARN(this->get_logger(), "Event rate control service is not available during raw file playback");
		response->success = false;
		return;
	}

	camera_settings_.event_rate_control_.setParams(request->enabled, request->rate);
	response->success = setEventRateControl();
}

void FluxeemRos2Publisher::setTrailFilterServiceCallback(
	const camera_interfaces::srv::SetTrailFilter::Request::SharedPtr request,
	camera_interfaces::srv::SetTrailFilter::Response::SharedPtr response)
{
	if (isRawFileMode())
	{
		RCLCPP_WARN(this->get_logger(), "Trail filter service is not available during raw file playback");
		response->success = false;
		return;
	}

	if (!camera_settings_.trail_filter_.setParams(request->enabled, request->threshold, request->filter_type))
	{
		RCLCPP_WARN(this->get_logger(), "Invalid trail filter type: %s", request->filter_type.c_str());
		response->success = false;
		return;
	}
	response->success = setTrailFilter();
}

void FluxeemRos2Publisher::recordServiceCallback(const camera_interfaces::srv::Record::Request::SharedPtr request,
												 camera_interfaces::srv::Record::Response::SharedPtr response)
{
	if (!camera_)
	{
		RCLCPP_WARN(this->get_logger(), "Recording request failed because the camera is not open");
		response->success = false;
		return;
	}

	try
	{
		if (request->enabled)
		{
			if (request->file_path.empty())
			{
				RCLCPP_WARN(this->get_logger(), "Recording request failed because file_path is empty");
				response->success = false;
				return;
			}

			response->success = camera_->start_recording(request->file_path);
			if (response->success)
			{
				recording_file_path_ = request->file_path;
				configureModuleReplayMetadata(recording_file_path_, "", -1);
			}
			return;
		}

		response->success = camera_->stop_recording();
		if (response->success)
		{
			recording_file_path_.clear();
			configureModuleReplayMetadata(recording_file_path_, "", -1);
		}
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN(this->get_logger(), "Recording request failed: %s", e.what());
		response->success = false;
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Recording request failed with an unknown error");
		response->success = false;
	}
}
