# State of the Autonomous System
uint16 as_state

uint16 AS_OFF = 0
uint16 AS_READY = 1
uint16 AS_DRIVING = 2
uint16 AS_EMERGENCY_BRAKE = 3
uint16 AS_FINISHED = 4

# Mission indicator
uint16 ami_state

uint16 AMI_NOT_SELECTED = 10
uint16 AMI_ACCELERATION = 11
uint16 AMI_SKIDPAD = 12
uint16 AMI_TRACK_DRIVE = 13
uint16 AMI_BRAKE_TEST = 14
uint16 AMI_INSPECTION = 15

# asserted true when mission can be started
bool mission_flag