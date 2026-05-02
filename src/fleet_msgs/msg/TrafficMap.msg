# Traffic Map Message
# Represents the complete traffic map with waypoints and routes

# Map metadata
string map_id
string map_name
string map_yaml_path  # Path to the original YAML map file

# List of all waypoints
Waypoint[] waypoints

# Creation timestamp
builtin_interfaces/Time created_at

# Last modification timestamp
builtin_interfaces/Time modified_at