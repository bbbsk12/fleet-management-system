# Robot Status Message
# Represents the status of a single robot

# Robot identification
string robot_id
string robot_namespace

# Connection status
string connection_status  # "online", "offline", "unknown"

# Battery information
float32 battery_percentage
bool battery_low  # True if battery < 20%

# Current position
geometry_msgs/Pose current_pose

# Current task
string current_task_id  # Empty if idle

# Navigation status
string nav_status  # "idle", "moving", "arrived", "failed", "executing"

# Current waypoint and route info
string location_type     # "waypoint", "segment", or "unknown"
string current_waypoint  # If at waypoint: waypoint ID
string current_segment   # If on segment: segment (e.g., "wp_001->wp_002")
string[] planned_route   # Full planned route waypoint IDs

# Timestamp of last update
builtin_interfaces/Time last_update