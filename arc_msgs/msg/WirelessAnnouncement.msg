std_msgs/Header header

#Where is the robot who sent this announcement? This is in /map coordinate frame.
geometry_msgs/Pose sender_location

#What kind of announcement is this
uint8 announcement_type
uint8 TYPE_INFO=0

arc_msgs/BotInfo info

#content of the announcement. The information to broadcast.
dynamic_reconfigure/Config announcement
