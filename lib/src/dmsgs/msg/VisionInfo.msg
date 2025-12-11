bool see_field
bool see_line
bool see_circle
bool see_corner
bool see_goal # todo remove this, if goals.size() == 0, then we didn't see goal. if goals.size() == 1, we saw unknown goal, if goal.size() == 2, goals[0] is left goal, ..
bool see_ball
bool see_obstacle
bool loc_ok
bool correct_yaw

float32 ball_quality
float32 field_quality
float32 field_consistency

# TODO(MWX): use vector for all, and remove bool values as much as possible
geometry_msgs/Vector3 robot_pos
geometry_msgs/Vector3 ball_field
geometry_msgs/Vector3 ball_global
geometry_msgs/Vector3 ball_velocity

geometry_msgs/Vector3 circle_field
geometry_msgs/Vector3 circle_global

geometry_msgs/Vector3 goal_field
geometry_msgs/Vector3 goal_global

geometry_msgs/Vector3[] obstacles_field
geometry_msgs/Vector3[] obstacles_global

FieldFeature[] features_field
# Global position of field corner. Used to specify which corner is detected
geometry_msgs/Vector3 corner_global

Line[] lines_field
geometry_msgs/Vector3[] viewRange
ParticleMsg[] particles

HeadCommand ballTrack
HeadCommand circleTrack
HeadCommand goalTrack
HeadCommand redGoalTrack # -450, 0
HeadCommand blueGoalTrack # 450, 0

BehaviorInfo behaviorInfo
