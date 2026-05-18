std_msgs/Header header
# Pose candidates are expressed in header.frame_id. MVP-1 expects base_link after TF conversion.
geometry_msgs/Pose grasp_pose
geometry_msgs/Pose cup_mouth_center
float32 confidence
string status
string source
