# Task Information Message
# Represents a task in the fleet management system

# Task identification
string task_id

# Target waypoint
string waypoint_id

# Target position
geometry_msgs/Pose target_pose

# Task type
# 1 = CRUISE, 2 = LOAD, 3 = UNLOAD, 4 = SITE_SPECIFIC
uint8 task_type

# Business code (meaning depends on task_type)
uint32 site_code

# Task status
# "pending", "assigned", "waiting_fleet", "in_progress", "executing", "completed", "failed", "cancelled"
string status

# Assigned robot (if any)
string assigned_robot_id  # Empty if not assigned

# Priority (higher number = higher priority)
int32 priority

# Task creation time
builtin_interfaces/Time created_at

# Task start time (when assigned)
builtin_interfaces/Time started_at

# Task completion time
builtin_interfaces/Time completed_at
