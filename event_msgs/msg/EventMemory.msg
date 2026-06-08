# This message contains the shared memory file identifier,
# and the size of the event messages

string identifier
uint64 length
float32 spatial_sample_rate
std_msgs/Header header
event_msgs/ReplayWindow replay_window