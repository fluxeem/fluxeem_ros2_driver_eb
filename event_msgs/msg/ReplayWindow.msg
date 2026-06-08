# Metadata required to reconstruct shared-memory payloads from a raw event file.
# raw_window_* values are camera-side event timestamps in microseconds.
# ROS message headers carry host-side receive time.

bool valid
string raw_file_path
string raw_file_hash
int64 raw_file_start_timestamp
int64 raw_window_start
int64 raw_window_end
uint16 roi_x_start
uint16 roi_x_end
uint16 roi_y_start
uint16 roi_y_end
int32 timespan
float64 keep_fraction
int32 temporal_sample_rate
int32 temporal_sample_rate_min
uint64 target_num_events_per_sec
int64 accumulation_start_timestamp
bool trigger_marked
int16 trigger_id
int16 trigger_polarity
int64 trigger_timestamp
