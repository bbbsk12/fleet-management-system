# Waypoint Message
# Represents a waypoint in the traffic map

# Waypoint ID
string waypoint_id

# Waypoint name (human-readable)
string name

# Position
geometry_msgs/Pose pose

# Connected waypoints (list of waypoint IDs)
string[] connections

# Waypoint properties
bool is_parking_spot  # True if this is a parking/rest location
bool is_charging_station  # True if this has charging capability
float32 radius  # Acceptance radius for reaching this waypoint