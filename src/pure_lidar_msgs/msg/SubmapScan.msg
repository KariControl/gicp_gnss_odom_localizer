# Filtered scan and the unmodified scan-to-scan pose for exactly one accepted
# odometer trajectory node. All four key fields are immutable across processes.
std_msgs/Header header
uint64 odom_session_id
uint64 odom_generation
uint64 sequence
geometry_msgs/PoseWithCovariance raw_pose
sensor_msgs/PointCloud2 cloud
