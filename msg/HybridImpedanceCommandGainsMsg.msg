std_msgs/Header header
# controller gains (position)
float64 kp_pos
float64 kp_att
float64 kp_gamma
float64 kd_pos
float64 kd_att
float64 kd_gamma
# controller gains (force)
float64 km_f
float64 kd_f
# internal motion gains 
float64 kp_z_im
float64 kp_gamma_im
float64 kd_pos_im
float64 kd_att_im
