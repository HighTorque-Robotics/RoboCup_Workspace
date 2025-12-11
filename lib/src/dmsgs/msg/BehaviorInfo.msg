# Decision Making
uint8 current_role
string skill

# Destination
geometry_msgs/Vector3 dest
geometry_msgs/Vector3 final_dest
geometry_msgs/Vector3 attack_target

# Misc
bool save_image
bool more_lines
bool vision_correct_yaw

# ROLE
uint8 ROLE_STRIKER    = 1
uint8 ROLE_DEFENDER   = 2
uint8 ROLE_SUPPORTER  = 3
uint8 ROLE_GOALIE     = 4
uint8 ROLE_OTHER      = 5

bool attack_right

# Playing state of robot
uint8 team_play_state

# Priority of current behavior
uint8 kicker_id
bool team_play_priority

bool[6] mates_online

float32 time_since_last_kick
