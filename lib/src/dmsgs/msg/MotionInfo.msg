bool stable
bool forward_or_backward
time timestamp
bool lower_board_connected
bool support_stablizer_flag

HeadCommand curPlat
geometry_msgs/Vector3 odometry
geometry_msgs/Vector3 imuRPY


uint8 status
uint8 STANDBY = 0
uint8 WALKING = 1
uint8 KICKING = 2

float64[] joint_feedback
float64[] neck_to_foot