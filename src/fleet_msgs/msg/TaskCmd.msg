# Task Command Message
# Sent by fleet_manager to chassis (OPi) to execute a task at the current location
# The chassis has already been navigated to the target waypoint by the scheduler
# Also used as ACK: fleet_manager sends TaskCmd with ack=true to confirm
# receipt of chassis TaskFb (HANDSHAKE_OK or COMPLETED or ERROR)

# Task identification (globally unique)
uint64 task_id

# Target waypoint index (pass-through to actuator, chassis does not validate)
uint16 waypoint_id

# Task type
# 1 = CRUISE  (arrive and complete, no actuator action)
# 2 = LOAD    (pick up cargo at waypoint)
# 3 = UNLOAD  (place cargo at waypoint)
# 4 = SITE_SPECIFIC (custom action defined by site_code)
uint8 task_type

# Business code (meaning depends on task_type)
# LOAD:       0x01=standard pick, 0x02=fragile pick
# UNLOAD:     0x01=standard place, 0x02=precision place
# SITE_SPECIFIC: 0x88=scan code, 0xFF=custom
uint32 site_code

# ACK flag: when true, this is an acknowledgement from fleet_manager
# confirming receipt of a TaskFb message (not a new task command).
# Chassis should stop resending TaskFb upon receiving this ACK.
bool ack
