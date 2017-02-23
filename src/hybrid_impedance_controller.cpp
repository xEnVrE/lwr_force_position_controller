#include <pluginlib/class_list_macros.h>
#include <lwr_force_position_controllers/hybrid_impedance_controller.h>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/WrenchStamped.h>

#define DEFAULT_KP 20
#define DEFAULT_KD 20
#define DEFAULT_KM_F 1
#define DEFAULT_KD_F 25
#define DEFAULT_CIRCLE_FREQ 0.1
#define DEFAULT_CIRCLE_RADIUS 0.1
#define DEFAULT_CIRCLE_CENTER_X 0
#define DEFAULT_CIRCLE_CENTER_Y 0
#define P2P_COEFF_3 10.0
#define P2P_COEFF_4 -15.0
#define P2P_COEFF_5 6.0
#define FORCE_REF_COEFF_3 -2
#define FORCE_REF_COEFF_2 3

namespace lwr_controllers {

  HybridImpedanceController::HybridImpedanceController() {}

  HybridImpedanceController::~HybridImpedanceController() {}
  
  bool HybridImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    // get parameters required by 
    // CartesianInverseDynamicsController::init(robot, n) from rosparam server 
    get_parameters(n);

    // this should be called *ONLY* after get_parameters 
    CartesianInverseDynamicsController::init(robot, n);

    // get publish rate from rosparam
    n.getParam("publish_rate", publish_rate_);
      
    // advertise HybridImpedanceCommand service
    set_cmd_service_ = n.advertiseService("set_hybrid_impedance_command",\
					  &HybridImpedanceController::set_cmd, this);
    get_cmd_service_ = n.advertiseService("get_hybrid_impedance_command",\
					  &HybridImpedanceController::get_cmd, this);
    
    // instantiate default controller gains
    Kp_ = Eigen::Matrix<double, 6, 6>::Identity() * DEFAULT_KP;
    Kd_ = Eigen::Matrix<double, 6, 6>::Identity() * DEFAULT_KD; 
    km_f_ = DEFAULT_KM_F;
    kd_f_ = DEFAULT_KD_F;

    // instantiate the desired trajectory
    x_des_ = Eigen::VectorXd(6);
    x_des_final_ = Eigen::VectorXd(2);
    xdot_des_ = Eigen::VectorXd(6);
    xdotdot_des_ = Eigen::VectorXd(6);

    // instantiate the matrix containing the constants
    // of the point to point trajectory
    p2p_trj_const_ = Eigen::MatrixXf(4, 2);

    //
    p2p_traj_duration_ = 5.0;
    is_first_iteration_p2p_traj_ = true;

    // of the point to point trajectory
    force_ref_const_ = Eigen::VectorXf(3);
    //
    force_ref_duration_ = 5.0;

    /////////////////////////////////////////////////
    // evaluate default trajectory
    /////////////////////////////////////////////////
    //

    double yaw_des, pitch_des, roll_des;
    KDL::Rotation R_des = KDL::Rotation::RotY(M_PI);
    R_des.GetEulerZYX(yaw_des, pitch_des, roll_des);

    // constant position
    x_des_final_(0) = 0;
    x_des_final_(1) = 0;
    x_des_ << 0, 0, 0.1, yaw_des, pitch_des, roll_des;
    xdot_des_ << 0, 0, 0, 0, 0, 0;
    xdotdot_des_ << 0, 0, 0, 0, 0, 0;
    
    // or circular trajectory (disabled by default)
    circle_trj_ = false;
    circle_trj_frequency_ = DEFAULT_CIRCLE_FREQ;
    circle_trj_radius_ = DEFAULT_CIRCLE_RADIUS;
    circle_trj_center_x_ = DEFAULT_CIRCLE_CENTER_X;
    circle_trj_center_y_ = DEFAULT_CIRCLE_CENTER_Y;
    time_ = 0;

    // init force reference coefficent the initial time and desired force
    fz_des_final_ = 0;
    for(int i = 0; i<3; i++)
      force_ref_const_(i)  = 0;
    time_force_ = 0;
    //
    ////////////////////////////////////////////////

    // advertise several topics
    pub_force_ = n.advertise<geometry_msgs::WrenchStamped>("force_measure", 1000);
    pub_force_des_ = n.advertise<geometry_msgs::WrenchStamped>("force_setpoint", 1000);
    pub_state_ = n.advertise<geometry_msgs::WrenchStamped>("state", 1000);
    pub_dstate_ = n.advertise<geometry_msgs::WrenchStamped>("dstate", 1000);
    pub_x_des_ = n.advertise<geometry_msgs::WrenchStamped>("x_des", 1000);
    pub_xdot_des_ = n.advertise<geometry_msgs::WrenchStamped>("xdot_des", 1000);
    pub_xdotdot_des_ = n.advertise<geometry_msgs::WrenchStamped>("xdotdot_des", 1000);
    pub_error_ = n.advertise<geometry_msgs::WrenchStamped>("error", 1000);

    return true;
  }

  void HybridImpedanceController::starting(const ros::Time& time)
  {
    CartesianInverseDynamicsController::starting(time);

    // initialize publish time
    last_publish_time_ = time;
  }

  void HybridImpedanceController::update(const ros::Time& time, const ros::Duration& period)
  {
    // evaluate inverse dynamics
    CartesianInverseDynamicsController::update(time, period);

    // the super class evaluates the following quantities every update
    //
    // ws_x_: distance and attitude between the workspace and the tool tip (workspace basis)
    // ws_xdot_: derivative of ws_x_ (workspace basis)
    // base_wrench_wrist_: forces and torques with the reference point on the wrist (world frame basis)
    // R_ws_base_: rotation matrix from workspace to vito_anchor
    // R_ws_ee: attitude of the end effector w.r.t to the workspace frame
    //
    
    // transform the ft_sensor wrench 
    // move the reference point from the wrist to the tool tip and project in workspace basis

    eval_force_reference(period);

    KDL::Frame force_transformation(R_ws_base_, R_ws_ee_ * (-p_wrist_ee_));
    KDL::Wrench ws_F_ee;
    ws_F_ee = force_transformation * base_wrench_wrist_;

    // check if the circular trajectory is requested by the user
    if (circle_trj_)
      eval_current_circular_traj(period);
    else
      eval_current_point_to_point_traj(period);

    // evaluate state error
    Eigen::VectorXd err_x = x_des_ - ws_x_;

    // normalize angular error between - M_PI and M_PI
    err_x(3) = angles::normalize_angle(err_x(3));
    err_x(4) = angles::normalize_angle(err_x(4));
    err_x(5) = angles::normalize_angle(err_x(5));

    /////////////////////////////////////////////////////////////////////
    //
    // desired acceleration
    //
    /////////////////////////////////////////////////////////////////////
    //

    Eigen::VectorXd acc_cmd = Eigen::VectorXd(6);

    // position controlled DoF
    // ws_x ws_y R_ee_ws_(yaw, pitch, roll) 
    acc_cmd = Kp_ * err_x + Kd_ * (xdot_des_ - ws_xdot_) + xdotdot_des_;
    acc_cmd(0) = acc_cmd(0) - ws_F_ee.force.x();
    acc_cmd(1) = acc_cmd(1) - ws_F_ee.force.y();
    acc_cmd(3) = acc_cmd(3);// - ws_F_ee.torque.x();
    acc_cmd(4) = acc_cmd(4);// - ws_F_ee.torque.y();
    acc_cmd(5) = acc_cmd(5);// - ws_F_ee.torque.z();

    // force controlled DoF
    // ws_Fz
    double err_force = fz_des_ - ws_F_ee.force.z();
    acc_cmd(2) = - kd_f_ * ws_xdot_(2) + km_f_ * err_force;

    //
    /////////////////////////////////////////////////////////////////////

    // call super class method set_command
    set_command(acc_cmd);

    if(time > last_publish_time_ + ros::Duration(1.0 / publish_rate_))
      {
	//update next tick
	last_publish_time_ += ros::Duration(1.0 / publish_rate_);

	// publish data
	publish_data(pub_state_, ws_x_);
	publish_data(pub_dstate_, ws_xdot_);
	publish_data(pub_x_des_, x_des_);
	publish_data(pub_xdot_des_, xdot_des_);
	publish_data(pub_xdotdot_des_, xdotdot_des_);
	publish_data(pub_force_, ws_F_ee);
	publish_data(pub_force_des_, KDL::Wrench(KDL::Vector(0, 0, fz_des_),\
						 KDL::Vector(0, 0, 0)));
	Eigen::VectorXd errors;
	errors = err_x;
	errors(2) = err_force;
	publish_data(pub_error_, errors);
      }
  }

  void HybridImpedanceController::get_parameters(ros::NodeHandle &n)
  {
    // vector from the wrist to the tool tip (projected in lwr_link_7 frame)
    std::vector<double> p_wrist_ee;

    // vector from vito_anchor to the workspace (projected in world frame)
    std::vector<double> p_base_ws;

    // attitude of the workspace frame w.r.t. vito_anchor frame
    std::vector<double> ws_base_angles;

    // get parameters form rosparameter server
    n.getParam("p_wrist_ee", p_wrist_ee);
    n.getParam("p_base_ws", p_base_ws);
    n.getParam("ws_base_angles", ws_base_angles);

    // set internal members
    set_p_wrist_ee(p_wrist_ee.at(0), p_wrist_ee.at(1), p_wrist_ee.at(2));
    set_p_base_ws(p_base_ws.at(0), p_base_ws.at(1), p_base_ws.at(2));
    set_ws_base_angles(ws_base_angles.at(0), ws_base_angles.at(1), ws_base_angles.at(2));
  }

  bool HybridImpedanceController::set_cmd(lwr_force_position_controllers::HybridImpedanceCommand::Request &req,\
					  lwr_force_position_controllers::HybridImpedanceCommand::Response &res)
  {
    // check if the user changed type of trajectory
    if (circle_trj_ != req.command.circle_trj)
      {
	time_ = 0;
	if(circle_trj_ == false)
	  is_first_iteration_p2p_traj_ = true;
      }
    // check if the set point changed
    if (x_des_final_(0) != req.command.x ||
	x_des_final_(1) != req.command.y)
      {
	time_ = 0;
	is_first_iteration_p2p_traj_ = true;
      }

    // set the desired final position and attitude requested by the user
    // position
    x_des_final_(0) = req.command.x;
    x_des_final_(1) = req.command.y;
    //p2p_traj_duration_ = req.command.p2p_traj_duration;

    // attitude
    x_des_(3) = req.command.yaw;
    x_des_(4) = req.command.pitch;
    x_des_(5) = req.command.roll;

    // reset the derivatives 
    xdot_des_(0) = 0;
    xdot_des_(1) = 0;
    xdotdot_des_(0) = 0;
    xdotdot_des_(1) = 0;
    
    // set the desired force
    evaluate_force_reference_constants(req.command.forcez);
    time_force_ = 0;    
    fz_des_final_ = req.command.forcez;

    // set desired parameters of the circular trajectory
    circle_trj_ = req.command.circle_trj;
    circle_trj_frequency_ = req.command.frequency;
    circle_trj_radius_ = req.command.radius;
    circle_trj_center_x_ = req.command.center_x;
    circle_trj_center_y_ = req.command.center_y;

    // set the desired gains requested by the user
    // TEMPORARY: -1 means that the user requested the last gain set
    if (req.command.kp != -1)
      Kp_ = Eigen::Matrix<double, 6, 6>::Identity() * req.command.kp;
    if (req.command.kd != -1)
      Kd_ = Eigen::Matrix<double, 6, 6>::Identity() * req.command.kd; 
    if (req.command.km_f != -1)
      km_f_ = req.command.km_f;
    if (req.command.kd_f != -1)
      kd_f_ = req.command.kd_f;

    return true;
  }

  bool HybridImpedanceController::get_cmd(lwr_force_position_controllers::HybridImpedanceCommand::Request &req,\
					  lwr_force_position_controllers::HybridImpedanceCommand::Response &res)
  {
    // get gains
    res.command.kp = Kp_(0, 0);
    res.command.kd = Kd_(0, 0);
    res.command.km_f = km_f_;
    res.command.kd_f = kd_f_;

    // get position
    res.command.x = x_des_final_(0);
    res.command.y = x_des_final_(1);
    res.command.p2p_traj_duration = p2p_traj_duration_;

    // get attitude
    res.command.yaw = x_des_(3);
    res.command.pitch = x_des_(4);
    res.command.roll = x_des_(5);
    
    // get force
    res.command.forcez = fz_des_final_;

    // get circle trajectory
    res.command.circle_trj = circle_trj_;
    res.command.frequency = circle_trj_frequency_;
    res.command.radius = circle_trj_radius_;
    res.command.center_x = circle_trj_center_x_;
    res.command.center_y = circle_trj_center_y_;

    return true;
  }

  void HybridImpedanceController::evaluate_force_reference_constants(double force_des)
  {
    force_ref_const_(0) = fz_des_final_;
    force_ref_const_(1) = FORCE_REF_COEFF_2 * (force_des - fz_des_final_) / pow(force_ref_duration_, 2);
    force_ref_const_(2) = FORCE_REF_COEFF_3 * (force_des - fz_des_final_) / pow(force_ref_duration_, 3);
    
  }

  void HybridImpedanceController::eval_force_reference(const ros::Duration& period)
  {
    time_force_ += period.toSec();
    if(time_force_ >= force_ref_duration_)
      time_force_ = force_ref_duration_;

    // q_des
    fz_des_ = force_ref_const_(2) * pow(time_force_, 3) + \
      force_ref_const_(1) * pow(time_force_, 2) + \
      force_ref_const_(0);
  }

  void HybridImpedanceController::eval_current_point_to_point_traj(const ros::Duration& period)
  {
    // this function is called inside the update() method
    // when this function is called the vector ws_x_ surely contains
    // the current state and eval_point_to_point_traj_constants()
    // can be called
    eval_point_to_point_traj_constants();

    time_ += period.toSec();
    
    if (time_ > p2p_traj_duration_)
      time_ = p2p_traj_duration_;

    for (int i=0; i<2; i++)
      {
	x_des_(i) = p2p_trj_const_(0, i) + p2p_trj_const_(1, i) * pow(time_, 3) + \
	  p2p_trj_const_(2, i) * pow(time_, 4) + p2p_trj_const_(3, i) * pow(time_, 5);

	xdot_des_(i) = 3 * p2p_trj_const_(1, i) * pow(time_, 2) + \
	  4 * p2p_trj_const_(2, i) * pow(time_, 3) + 5 * p2p_trj_const_(3, i) * pow(time_, 4);

	xdotdot_des_(i) = 3 * 2 *  p2p_trj_const_(1, i) * time_ + \
	  4 * 3 * p2p_trj_const_(2, i) * pow(time_, 2) + 5 * 4 * p2p_trj_const_(3, i) * pow(time_, 3);
      }

  }

  void HybridImpedanceController::eval_point_to_point_traj_constants()
  { 
    if (is_first_iteration_p2p_traj_)
      {
	// evaluate common part of constants
	double constant_0, constant_1, constant_2;
	constant_0 = P2P_COEFF_3 / pow(p2p_traj_duration_, 3);
	constant_1 = P2P_COEFF_4 / pow(p2p_traj_duration_, 4);
	constant_2 = P2P_COEFF_5 / pow(p2p_traj_duration_, 5);

	// evaluate constants for x and y trajectories
	for (int i=0; i<2; i++)
	  {
	    double error = x_des_final_(i) - ws_x_(i);
	    p2p_trj_const_(0, i) = ws_x_(i);
	    p2p_trj_const_(1, i) = error * constant_0;
	    p2p_trj_const_(2, i) = error * constant_1;
	    p2p_trj_const_(3, i) = error * constant_2;
	  }
	is_first_iteration_p2p_traj_ = false;
      }
  }

  void HybridImpedanceController::eval_current_circular_traj(const ros::Duration& period)
  {
    // evaluate the circular trajectory
    time_ = time_ + period.toSec();
    double f = circle_trj_frequency_;
    double omega = 2 * M_PI * f;
    double rho = circle_trj_radius_;
    double x_trj = circle_trj_center_x_ + rho * cos(omega * time_);
    double y_trj = circle_trj_center_y_ + rho * sin(omega * time_);
    double dx_trj = -omega * rho * sin(omega * time_);
    double dy_trj = omega * rho * cos(omega * time_);
    double ddx_trj = -omega * omega * rho * cos(omega * time_);
    double ddy_trj = -omega * omega * rho * sin(omega * time_);

    // set the position
    x_des_(0) = x_trj;
    x_des_(1) = y_trj;

    // set the velocity
    xdot_des_(0) = dx_trj;
    xdot_des_(1) = dy_trj;

    // set the acceleration
    xdotdot_des_(0) = ddx_trj;
    xdotdot_des_(1) = ddy_trj;
  }

  void HybridImpedanceController::publish_data(ros::Publisher& pub, KDL::Wrench wrench)
  {
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.x = wrench.force.x(); 
    wrench_msg.wrench.force.y = wrench.force.y(); 
    wrench_msg.wrench.force.z = wrench.force.z(); 
    wrench_msg.wrench.torque.x = wrench.torque.x(); 
    wrench_msg.wrench.torque.y = wrench.torque.y(); 
    wrench_msg.wrench.torque.z = wrench.torque.z();
    pub.publish(wrench_msg);
  }

  void HybridImpedanceController::publish_data(ros::Publisher& pub, Eigen::VectorXd& vector)
  {
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.wrench.force.x = vector(0);
    msg.wrench.force.y = vector(1);
    msg.wrench.force.z = vector(2);
    msg.wrench.torque.x = vector(3);
    msg.wrench.torque.y = vector(4);
    msg.wrench.torque.z = vector(5);
    pub.publish(msg);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::HybridImpedanceController, controller_interface::ControllerBase)
