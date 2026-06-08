#pragma once

#include <algorithm>
#include <string>

#define BIAS_FO_MIN_OFFSET -20
#define BIAS_FO_MAX_OFFSET 0
#define BIAS_HPF_MIN_OFFSET 0
#define BIAS_HPF_MAX_OFFSET 120
#define BIAS_DIFF_ON_MIN_OFFSET -80
#define BIAS_DIFF_ON_MAX_OFFSET 145
#define BIAS_DIFF_MIN_OFFSET -25
#define BIAS_DIFF_MAX_OFFSET 23
#define BIAS_DIFF_OFF_MIN_OFFSET -30
#define BIAS_DIFF_OFF_MAX_OFFSET 200
#define BIAS_REFR_MIN_OFFSET -20
#define BIAS_REFR_MAX_OFFSET 235

#define EVENT_RATE_MEV_MAX 320  //320 MEv/s max
#define EVENT_RATE_MEV_MIN 0
#define EVENT_RATE_MEV_DEFAULT 320

struct Biases
{
	int bias_diff_;
	int bias_diff_on_;
	int bias_diff_off_;
	int bias_fo_;
	int bias_hpf_;
	int bias_refr_;

	bool setParams(int bias_diff, int bias_diff_on, int bias_diff_off, int bias_fo, int bias_hpf, int bias_refr)
	{
		bias_diff_ = std::clamp(bias_diff, BIAS_DIFF_MIN_OFFSET, BIAS_DIFF_MAX_OFFSET);
		bias_diff_off_ = std::clamp(bias_diff_off, BIAS_DIFF_OFF_MIN_OFFSET, BIAS_DIFF_OFF_MAX_OFFSET);
		bias_diff_on_ = std::clamp(bias_diff_on, BIAS_DIFF_ON_MIN_OFFSET, BIAS_DIFF_ON_MAX_OFFSET);
		bias_fo_ = std::clamp(bias_fo, BIAS_FO_MIN_OFFSET, BIAS_FO_MAX_OFFSET);
		bias_hpf_ = std::clamp(bias_hpf, BIAS_HPF_MIN_OFFSET, BIAS_HPF_MAX_OFFSET);
		bias_refr_ = std::clamp(bias_refr, BIAS_REFR_MIN_OFFSET, BIAS_REFR_MAX_OFFSET);

		return true;
	}
};

struct TrailFilter
{
	bool enable_;
	int threshold_;
	std::string type_;

	bool setParams(bool enable, int threshold, const std::string &type)
	{
		enable_ = enable;
		threshold_ = threshold;

		if (type == "STC_CUT_TRAIL")
		{
			type_ = "STC_CUT_TRAIL";
		}
		else if (type == "STC_KEEP_TRAIL")
		{
			type_ = "STC_KEEP_TRAIL";
		}
		else if (type == "TRAIL")
		{
			type_ = "TRAIL";
		}
		else
		{
			return false;
		}

		return true;
	}
};

struct EventRateControl
{
	bool enable_;
	int rate_;

	bool setParams(bool enable, int rate)
	{
		enable_ = enable;
		rate_ = std::clamp(rate, EVENT_RATE_MEV_MIN, EVENT_RATE_MEV_MAX);

		return true;
	}
};

struct CameraSettings
{
	Biases biases_;
	TrailFilter trail_filter_;
	EventRateControl event_rate_control_;
};