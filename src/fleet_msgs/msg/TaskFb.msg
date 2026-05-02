# Task Feedback Message
# Sent by chassis (OPi) to fleet_manager to report task execution status

# Task identification (must match TaskCmd.task_id)
uint64 task_id

# Task status
# 0 = HANDSHAKE_OK  (chassis received task, starting execution)
# 1 = COMPLETED     (task finished successfully)
# 2 = ERROR         (task failed / timeout / interrupted)
uint8 status

# Error code (only meaningful when status=2)
# 0x01 = invalid parameters
# 0x02 = serial comm failure (OPi<->STM32)
# 0x03 = actuator no response (STM32<->actuator)
# 0x04 = action execution failed (e.g. grip slipped)
# 0x05 = action execution timeout (30s)
# 0x00 = unknown error
uint8 error_code
