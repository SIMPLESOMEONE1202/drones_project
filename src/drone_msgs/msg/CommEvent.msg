# One logged communication event between two drones (or drone <-> base station)
std_msgs/Header header

string sender_id
string receiver_id           # "BASE" for centralized-mode messages to/from the ground station

string event_type            # HANDSHAKE, HEARTBEAT, INTRUDER_ALERT, STATE_SYNC, DROPOUT, REJOIN

string protocol_mode         # CENTRALIZED or DECENTRALIZED

float64 distance              # metres between sender and receiver at time of event
float64 latency_ms            # simulated/measured message latency
float64 battery_cost_percent  # battery drained by this drone for sending this message

bool success                  # false if out of range / message failed to deliver