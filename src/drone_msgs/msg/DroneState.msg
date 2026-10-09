# Full telemetry state for a single drone
std_msgs/Header header
string drone_id

# Position (world frame, metres)
float64 x
float64 y
float64 z

# Attitude (radians)
float64 roll
float64 pitch
float64 yaw

# Velocity (m/s, body or world frame — we'll fix this convention in drone_control)
float64 vx
float64 vy
float64 vz

# Power
float64 battery_percent      # 0.0 - 100.0
float64 battery_voltage      # volts, optional finer-grained model later

# Mission state machine: PATROL, INTERCEPT, RTB, REJOIN
string mission_state

# Comms
bool comms_active            # false if this drone has dropped out of range of the swarm
int32 known_peers            # how many other drones this one currently has in range