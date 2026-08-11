# Full SE(2) precision observation for exactly one SubmapScan key.
std_msgs/Header header
uint64 odom_session_id
uint64 odom_generation
uint64 sequence
# A matcher restart changes matcher_session_id and restarts its transform near
# identity.  Consumers must rebase the first correction of a new matcher
# session onto their last applied precision transform to avoid an output jump.
uint64 matcher_session_id
uint64 submap_generation
uint64 correction_id
# Target frame of precision_from_raw and corrected_pose.  header.frame_id is
# the source raw-odom frame copied from the exact target SubmapScan.
string precision_frame_id
# Persistent full-SE(2) transform precision_odom <- raw_odom.  A consumer can
# compose this directly with any later raw odometry pose; z/roll/pitch are zero.
geometry_msgs/Transform precision_from_raw
# The same transform applied to the exact target SubmapScan raw_pose.  Its
# covariance supplies the accepted observation uncertainty at that scan.
geometry_msgs/PoseWithCovariance corrected_pose
bool use_yaw
float64 fitness
float64 inlier_ratio
# Registered pose minus the previous committed prediction at the vehicle pivot.
float64 innovation_translation_m
float64 innovation_x_m
float64 innovation_y_m
float64 innovation_yaw_rad
# Number of mutually consistent measurements used for this committed update.
uint32 consistency_count
