std_msgs/Header header

#Where is the robot who sent this announcement? This is in /map coordinate frame.
geometry_msgs/Pose sender_location

#All broadcasts contain info of sending robot
arc_msgs/BotInfo info

#Who are we sending this response to
uuid_msgs/UniqueID target_id

#What type of response is this
uint8 type
uint8 TASK_RESPONSE = 0
uint8 TASK_CONFIRMATION = 1
uint8 TASK_ACKNOWLEDGEMENT = 2

#The contents of the response
dynamic_reconfigure/Config response
