# LED Task Message
# Sent by fleet_manager to chassis (OPi) at 5Hz to control LED strip

uint8 STATE_WALKING=0          # Walking / navigating
uint8 STATE_TRAFFIC_WAIT=1     # Waiting for traffic clearance
uint8 STATE_TASK_EXECUTING=2   # Executing chassis task (LOAD/UNLOAD/etc.)
uint8 STATE_IDLE=3             # Idle, no active task

uint8 state  # Current system state (0~3)
