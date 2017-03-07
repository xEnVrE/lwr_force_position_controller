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
float64 z
float64 p2p_traj_duration
# desired ZYZ
float64 alpha
float64 beta
float64 gamma
# desired force
float64 forcez
float64 force_ref_duration
bool enable_force

