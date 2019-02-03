# Output of system state
std_msgs/Header header

# possible statuses
byte INIT=0		 	# Initilisation mode; waiting for command
byte MAPPING=1   	# Mapping mode; only for track drive
byte RUNNING=2   	# Mapping has finished, running full speed
byte LOST=3  	 	# SLAM algorithms have been lost after finish of mapping
byte EMERGENCY=4 	# Something bad happened; enable emergency brakes
byte COMPLETED=5 	# Mission completed

byte status # status types enumerated above

# laps done, only for track drive event
uint16 laps
