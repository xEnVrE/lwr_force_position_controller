std_msgs/Header header
# controller gains (position)
float64 kp
float64 kd
# controller gains (force)
float64 km_f
float64 kd_f
# desired position
float64 x
float64 y
float64 p2p_traj_duration
# desired YPR (aka ZYX)
float64 yaw
float64 pitch
float64 roll
# desired force
float64 forcez
#circle trajectory
bool circle_trj
float64 frequency
float64 radius
float64 center_x
float64 center_y
