#include <fluxeem_ros2_publisher/fluxeem_ros2_publisher.h>

#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_event_trail_filter_module.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_trigger_in.h>

#include <cstdint>
#include <exception>
#include <map>

namespace
{
constexpr uint32_t EVENTS_PER_MEV = 1000000U;

Metavision::I_EventTrailFilterModule::Type trailFilterTypeFromString(const std::string &type)
{
	if (type == "STC_CUT_TRAIL")
	{
		return Metavision::I_EventTrailFilterModule::Type::STC_CUT_TRAIL;
	}
	if (type == "STC_KEEP_TRAIL")
	{
		return Metavision::I_EventTrailFilterModule::Type::STC_KEEP_TRAIL;
	}
	return Metavision::I_EventTrailFilterModule::Type::TRAIL;
}

std::string trailFilterTypeToString(Metavision::I_EventTrailFilterModule::Type type)
{
	switch (type)
	{
	case Metavision::I_EventTrailFilterModule::Type::STC_CUT_TRAIL:
		return "STC_CUT_TRAIL";
	case Metavision::I_EventTrailFilterModule::Type::STC_KEEP_TRAIL:
		return "STC_KEEP_TRAIL";
	case Metavision::I_EventTrailFilterModule::Type::TRAIL:
	default:
		return "TRAIL";
	}
}
} // namespace

bool FluxeemRos2Publisher::setBiases()
{
	if (!camera_)
	{
		return false;
	}

	try
	{
		auto *biases = camera_->get_device().get_facility<Metavision::I_LL_Biases>();
		if (!biases)
		{
			RCLCPP_WARN(this->get_logger(), "Bias facility is not available");
			return false;
		}

		bool result = biases->set("bias_diff", camera_settings_.biases_.bias_diff_);
		result &= biases->set("bias_diff_on", camera_settings_.biases_.bias_diff_on_);
		result &= biases->set("bias_diff_off", camera_settings_.biases_.bias_diff_off_);
		result &= biases->set("bias_fo", camera_settings_.biases_.bias_fo_);
		result &= biases->set("bias_hpf", camera_settings_.biases_.bias_hpf_);
		result &= biases->set("bias_refr", camera_settings_.biases_.bias_refr_);

		return result;
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set biases: %s", e.what());
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set biases with an unknown error");
	}

	return false;
}

bool FluxeemRos2Publisher::getBiasesInfo(int &bias_diff, int &bias_diff_on, int &bias_diff_off, int &bias_fo,
										 int &bias_hpf, int &bias_refr)
{
	if (!camera_)
	{
		return false;
	}

	try
	{
		auto *biases = camera_->get_device().get_facility<Metavision::I_LL_Biases>();
		if (!biases)
		{
			RCLCPP_WARN(this->get_logger(), "Bias facility is not available");
			return false;
		}

		const std::map<std::string, int> all_biases = biases->get_all_biases();
		bool result = true;
		auto read_bias = [&](const std::string &name, int &value)
		{
			const auto iter = all_biases.find(name);
			if (iter == all_biases.end())
			{
				value = 0;
				result = false;
				return;
			}
			value = iter->second;
		};

		read_bias("bias_diff", bias_diff);
		read_bias("bias_diff_on", bias_diff_on);
		read_bias("bias_diff_off", bias_diff_off);
		read_bias("bias_fo", bias_fo);
		read_bias("bias_hpf", bias_hpf);
		read_bias("bias_refr", bias_refr);

		return result;
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to read biases: %s", e.what());
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to read biases with an unknown error");
	}

	return false;
}

bool FluxeemRos2Publisher::setEventRateControl()
{
	if (!camera_)
	{
		return false;
	}

	try
	{
		auto *erc = camera_->get_device().get_facility<Metavision::I_ErcModule>();
		if (!erc)
		{
			RCLCPP_WARN(this->get_logger(), "Event rate control facility is not available");
			return false;
		}

		const uint32_t events_per_sec =
			static_cast<uint32_t>(camera_settings_.event_rate_control_.rate_) * EVENTS_PER_MEV;
		bool result = erc->set_cd_event_rate(events_per_sec);
		result &= erc->enable(camera_settings_.event_rate_control_.enable_);
		return result;
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set event rate control: %s", e.what());
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set event rate control with an unknown error");
	}

	return false;
}

bool FluxeemRos2Publisher::getEventRateControlInfo(bool &enable, int &rate)
{
	if (!camera_)
	{
		return false;
	}

	try
	{
		auto *erc = camera_->get_device().get_facility<Metavision::I_ErcModule>();
		if (!erc)
		{
			RCLCPP_WARN(this->get_logger(), "Event rate control facility is not available");
			return false;
		}

		enable = erc->is_enabled();
		rate = static_cast<int>(erc->get_cd_event_rate() / EVENTS_PER_MEV);
		return true;
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to read event rate control: %s", e.what());
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to read event rate control with an unknown error");
	}

	return false;
}

bool FluxeemRos2Publisher::setTrailFilter()
{
	if (!camera_)
	{
		return false;
	}

	if (camera_settings_.trail_filter_.threshold_ < 0)
	{
		RCLCPP_WARN(this->get_logger(), "Trail filter threshold must be non-negative");
		return false;
	}

	try
	{
		auto *trail_filter = camera_->get_device().get_facility<Metavision::I_EventTrailFilterModule>();
		if (!trail_filter)
		{
			RCLCPP_WARN(this->get_logger(), "Trail filter facility is not available");
			return false;
		}

		const auto filter_type = trailFilterTypeFromString(camera_settings_.trail_filter_.type_);
		const auto available_types = trail_filter->get_available_types();
		if (available_types.find(filter_type) == available_types.end())
		{
			RCLCPP_WARN(this->get_logger(), "Trail filter type %s is not available",
						camera_settings_.trail_filter_.type_.c_str());
			return false;
		}

		bool result = trail_filter->set_threshold(static_cast<uint32_t>(camera_settings_.trail_filter_.threshold_ * 1000));
		result &= trail_filter->set_type(filter_type);
		result &= trail_filter->enable(camera_settings_.trail_filter_.enable_);
		return result;
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set trail filter: %s", e.what());
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set trail filter with an unknown error");
	}

	return false;
}

bool FluxeemRos2Publisher::getTrailFilterInfo(bool &enable, std::string &type, int &threshold)
{
	if (!camera_)
	{
		return false;
	}

	try
	{
		auto *trail_filter = camera_->get_device().get_facility<Metavision::I_EventTrailFilterModule>();
		if (!trail_filter)
		{
			RCLCPP_WARN(this->get_logger(), "Trail filter facility is not available");
			return false;
		}

		enable = trail_filter->is_enabled();
		type = trailFilterTypeToString(trail_filter->get_type());
		threshold = static_cast<int>(trail_filter->get_threshold());
		return true;
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to read trail filter: %s", e.what());
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to read trail filter with an unknown error");
	}

	return false;
}

void FluxeemRos2Publisher::setCamera()
{
	try
	{
		if (raw_file_.empty())
		{
			setTrailFilter();
		}
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set the trail filter");
	}

	try
	{
		if (raw_file_.empty())
		{
			setEventRateControl();
		}
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set the event rate control");
	}

	try
	{
		if (raw_file_.empty())
		{
			setBiases();
		}
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to set the biases");
	}

	try
	{
		if (raw_file_.empty() && isTriggerPublishMode())
		{
			auto *trigger_in = camera_->get_device().get_facility<Metavision::I_TriggerIn>();
			if (!trigger_in)
			{
				RCLCPP_WARN(this->get_logger(), "Trigger input facility is not available");
				return;
			}

			const auto available_channels = trigger_in->get_available_channels();
			if (available_channels.find(Metavision::I_TriggerIn::Channel::Main) == available_channels.end())
			{
				RCLCPP_WARN(this->get_logger(), "Main trigger input channel is not available");
				return;
			}

			if (!trigger_in->enable(Metavision::I_TriggerIn::Channel::Main))
			{
				RCLCPP_WARN(this->get_logger(), "Failed to enable trigger input");
			}
		}
	}
	catch (...)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to configure trigger input");
	}
}
