# Dedicated GNSS observation for map/odom fusion.
#
# header.stamp is the GNSS observation time. When has_odom=true, odom stores
# the synchronized absolute observation in header.frame_id (normally map).
# The position can be either base_link or a GNSS antenna observation point.
# Orientation, when heading_valid=true, always represents base_link yaw.
# Consumers must never infer heading validity from a placeholder quaternion.
#
# For a single antenna:
#   position_is_base_link=false
#   observation_point_valid=true
#   observation_point_in_base = base_link -> antenna lever arm
#
# For a derived dual-antenna base pose:
#   position_is_base_link=true
#   observation_point_valid=false
#
# The x/y/yaw covariance and cross-covariance are carried in odom.pose.covariance.

std_msgs/Header header
bool has_odom
nav_msgs/Odometry odom
int8 fix_status
int8 fix_quality -1   # GGA quality: 0 invalid, 1 GPS, 2 DGPS, 4 RTK FIX, 5 RTK FLOAT
float32 confidence    # [0, 1], valid only when has_confidence=true
bool has_confidence
bool gnss_usable
bool heading_valid
string heading_source
bool position_is_base_link
bool observation_point_valid
geometry_msgs/Vector3 observation_point_in_base
