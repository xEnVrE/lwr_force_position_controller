std_msgs/Header header
# controller gains
float64 kp
float64 kd
# desired position
float64 x
float64 y
float64 z
# desired YPR (aka ZYX)
float64 yaw
float64 pitch
float64 roll
bool hold_last_qdes_found
# duration of trajectory
float64 p2p_traj_duration