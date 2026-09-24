# Text to display on the ground control station, sent as a MAVLink STATUSTEXT.
# The fields mirror mavros_msgs/msg/StatusText.
# https://mavlink.io/en/messages/common.html#STATUSTEXT

# Severity levels, from MAV_SEVERITY.
uint8 EMERGENCY = 0
uint8 ALERT = 1
uint8 CRITICAL = 2
uint8 ERROR = 3
uint8 WARNING = 4
uint8 NOTICE = 5
uint8 INFO = 6
uint8 DEBUG = 7

std_msgs/Header header

uint8 severity # Note that the default of zero is EMERGENCY, so always set it.
string text # Sent as-is, longer text is split over several STATUSTEXT messages.
