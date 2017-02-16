std_msgs/Header header
# controller gains
float64 kp
float64 kd
# desired
float64 x
float64 y
float64 z
# desired YPR (aka ZYX)
float64 yaw
float64 pitch
float64 roll
# desired controller strategy
bool use_inverse_dynamics_controller