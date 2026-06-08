# This message contains the shared memory file identifier,
# and the size of the event messages

string identifier
uint32 height
uint32 width
uint8 channel
std_msgs/Header header
event_msgs/ReplayWindow replay_window