# LED Status Message
# Sent by chassis (OPi) to fleet_manager at 5Hz

uint8 state     # Current actual LED state (0~3, 255=DISCONNECTED)
bool  received  # Whether chassis has ever received an LEDTask from fleet_manager
