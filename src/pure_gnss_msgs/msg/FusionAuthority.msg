# Event-driven machine contract exported by GNSS map/odom fusion.
#
# FULL_SE2_HEALTHY is the only state that permits a downstream consumer to
# update a global anchor. SOFT_BAD_HOLD is deliberately unhealthy: the fusion
# node may preserve its internal recovery window, but consumers must freeze
# their last trusted anchor and must not consume a new authority candidate.

uint8 UNHEALTHY=0
uint8 FULL_SE2_HEALTHY=1
uint8 SOFT_BAD_HOLD=2

uint8 FIX_UNKNOWN=0
uint8 FIX_GOOD=1
uint8 FIX_BAD=2

std_msgs/Header header
builtin_interfaces/Time source_stamp
uint64 session_id
uint64 sequence
uint8 state
string reason
string recovery_state
bool anchor_valid
bool position_fused
bool yaw_fused
uint8 last_fix_state
