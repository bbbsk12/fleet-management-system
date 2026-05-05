# Fleet Status Message
# Represents the status of the entire fleet

# Timestamp
builtin_interfaces/Time timestamp

# List of robot statuses
RobotStatus[] robots

# List of pending tasks
TaskInfo[] pending_tasks

# Total number of active tasks
int32 active_tasks

# System health status
string system_status  # "healthy", "warning", "error"