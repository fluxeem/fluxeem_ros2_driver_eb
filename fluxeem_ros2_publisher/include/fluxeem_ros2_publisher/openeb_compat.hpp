#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/events/event_ext_trigger.h>
#include <metavision/sdk/base/utils/timestamp.h>

using Timestamp = Metavision::timestamp;

struct Event
{
	uint16_t x;
	uint16_t y;
	int16_t polarity;
	Timestamp timestamp;
};

struct TriggerEvent
{
	int16_t polarity;
	int16_t id;
	Timestamp timestamp;
};

using EventBatch = std::vector<Event>;

inline Event convertEvent(const Metavision::EventCD &event)
{
	return Event{event.x, event.y, static_cast<int16_t>(event.p), event.t};
}

inline TriggerEvent convertTriggerEvent(const Metavision::EventExtTrigger &event)
{
	return TriggerEvent{static_cast<int16_t>(event.p), static_cast<int16_t>(event.id), event.t};
}

inline EventBatch convertEventBatch(const Metavision::EventCD *begin, const Metavision::EventCD *end)
{
	EventBatch events;
	events.reserve(static_cast<std::size_t>(end - begin));
	for (const Metavision::EventCD *event = begin; event != end; event++)
	{
		events.push_back(convertEvent(*event));
	}
	return events;
}
