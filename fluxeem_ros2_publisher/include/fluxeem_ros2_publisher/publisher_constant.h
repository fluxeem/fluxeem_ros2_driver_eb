#pragma once

#define QOS_QUEUE 8 // The QoS queue size for the publisher

#define CLEAN_TIME_SPAN_SEC 2.0 // The time span for cleaning the shared memory file, the unit is seconds
#define REST_TIME_SPAN                                                                                                 \
	1000 // The time span for restart all the work after successfully reopening camera, the unit is milliseconds
#define SEC_TO_MICROSEC 1'000'000 // The time conversion from sec to microsec

#define BG_COLOR 128	   // The background color of the time surface, the value should be in the range of [0, 255]
#define POSITIVE_COLOR 255 // The color of positive events, the value should be in the range of [0, 255]
#define NEGATIVE_COLOR 0   // The color of negative events, the value should be in the range of [0, 255]