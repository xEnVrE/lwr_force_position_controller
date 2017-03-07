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
#define DEFAULT_P2P_TRAJ_DURATION 5.0
#define DEFAULT_FORCE_TRAJ_DURATION 2.0
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

    // instantiate variables
    Kp_ = Eigen::Matrix<double, 6, 6>::Identity() * DEFAULT_KP;
    Kd_ = Eigen::Matrix<double, 6, 6>::Identity() * DEFAULT_KD; 
    p2p_trj_const_ = Eigen::MatrixXf(4, 6);
    force_ref_const_ = Eigen::VectorXf(3);

    // set defaults
    km_f_ = DEFAULT_KM_F;
    kd_f_ = DEFAULT_KD_F;
    circle_trj_ = false;
    circle_trj_frequency_ = DEFAULT_CIRCLE_FREQ;
    circle_trj_radius_ = DEFAULT_CIRCLE_RADIUS;
    circle_trj_center_x_ = DEFAULT_CIRCLE_CENTER_X;
    circle_trj_center_y_ = DEFAULT_CIRCLE_CENTER_Y;
    p2p_traj_duration_ = DEFAULT_P2P_TRAJ_DURATION;
    force_ref_duration_ = DEFAULT_FORCE_TRAJ_DURATION;

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

    ///////////////////////////////////////////////////////////////
    //
    // every time the controller is started the set point is set
    // to the current configuration
    //
    ///////////////////////////////////////////////////////////////
    //

    // get current robot joints configuration q
    KDL::JntArray q;
    q.resize(kdl_chain_.getNrOfJoints());
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
	q(i) = joint_handles_[i].getPosition();

    // forward kinematics
    KDL::Frame ee_fk_frame;
    ee_fk_solver_->JntToCart(q, ee_fk_frame);

    // evaluate current cartesian configuration
    KDL::Rotation R_ws_ee;
    KDL::Vector p_ws_ee;
    double alpha, beta, gamma;
    R_ws_ee = R_ws_base_ * ee_fk_frame.M;
    R_ws_ee.GetEulerZYZ(alpha, beta, gamma);
    p_ws_ee = R_ws_base_ * (ee_fk_frame.p - p_base_ws_);

    // set position and attitude tajectory constants
    for(int i=0; i<6; i++)
      {
	p2p_trj_const_(1, i) = 0;
	p2p_trj_const_(2, i) = 0;
	p2p_trj_const_(3, i) = 0;
      }

    for(int i=0; i<2; i++)
      p2p_trj_const_(0, i) = p_ws_ee.data[i];
    p2p_trj_const_(0, 3) = alpha;
    p2p_trj_const_(0, 4) = beta;
    p2p_trj_const_(0, 5) = gamma;
    prev_pos_setpoint_ << p_ws_ee.x(), p_ws_ee.y(), 0.2;
    prev_att_setpoint_ << alpha, beta, gamma;

    // set force trajectory constants
    for(int i = 0; i<3; i++)
      force_ref_const_(i)  = 0;
    force_ref_const_(0) = -0.1;
    prev_fz_setpoint_ = -0.1;

    // reset the time
    time_ = 0;
    time_force_ = 0;

    //
    ///////////////////////////////////////////////////////////////
  }

  void HybridImpedanceController::update(const ros::Time& time, const ros::Duration& period)
  {
    // evaluate inverse dynamics
    CartesianInverseDynamicsController::update(time, period);

    // the super class evaluates the following quantities every update
    //
    // ws_x_: distance and attitude between the workspace and the tool tip (workspace basis)
    // ws_xdot_: derivative of ws_x_ (workspace basis)
    // wrench_wrist_: forces and torques with the reference point on the wrist (ee frame basis)
    // R_ws_base_: rotation matrix from workspace to vito_anchor
    // R_ws_ee: attitude of the end effector w.r.t to the workspace frame
    //
    
    // transform the ft_sensor wrench 
    // move the reference point from the wrist to the tool tip and project in workspace basis

    KDL::Frame force_transformation(R_ws_ee_, R_ws_ee_ * (-p_sensor_cp_));
    KDL::Wrench ws_F_ee;
    ws_F_ee = force_transformation * wrench_wrist_;

    Eigen::VectorXd x_des(6);
    Eigen::VectorXd xdot_des(6);
    Eigen::VectorXd xdotdot_des(6);
    double fz_des;
    eval_current_point_to_point_traj(period, x_des, xdot_des, xdotdot_des);
    fz_des = eval_force_reference(period);

    // evaluate state error
    Eigen::VectorXd err_x = x_des - ws_x_;

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
    // ws_x ws_y R_ee_ws_(alpha, beta, gamma) 
    acc_cmd = Kp_ * err_x + Kd_ * (xdot_des - ws_xdot_) + xdotdot_des;
    acc_cmd(0) = acc_cmd(0) - ws_F_ee.force.x();
    acc_cmd(1) = acc_cmd(1) - ws_F_ee.force.y();
    acc_cmd(3) = acc_cmd(3);// - ws_F_ee.torque.x();
    acc_cmd(4) = acc_cmd(4);// - ws_F_ee.torque.y();
    acc_cmd(5) = acc_cmd(5);// - ws_F_ee.torque.z();

    // force controlled DoF
    // ws_Fz
    double err_force = fz_des - ws_F_ee.force.z();
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
	publish_data(pub_x_des_, x_des);
	publish_data(pub_xdot_des_, xdot_des);
	publish_data(pub_xdotdot_des_, xdotdot_des);
	publish_data(pub_force_, ws_F_ee);
	publish_data(pub_force_des_, KDL::Wrench(KDL::Vector(0, 0, fz_des),\
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

    // vector from the sensor to the contact point
    std::vector<double> p_sensor_cp;

    // vector from vito_anchor to the workspace (projected in world frame)
    std::vector<double> p_base_ws;

    // attitude of the workspace frame w.r.t. vito_anchor frame
    std::vector<double> ws_base_angles;

    // get parameters form rosparameter server
    n.getParam("p_wrist_ee", p_wrist_ee);
    n.getParam("p_sensor_cp", p_sensor_cp);
    n.getParam("p_base_ws", p_base_ws);
    n.getParam("ws_base_angles", ws_base_angles);

    // set internal members
    set_p_wrist_ee(p_wrist_ee.at(0), p_wrist_ee.at(1), p_wrist_ee.at(2));
    set_p_sensor_cp(p_sensor_cp.at(0), p_sensor_cp.at(1), p_sensor_cp.at(2));
    set_p_base_ws(p_base_ws.at(0), p_base_ws.at(1), p_base_ws.at(2));
    set_ws_base_angles(ws_base_angles.at(0), ws_base_angles.at(1), ws_base_angles.at(2));
  }

  void HybridImpedanceController::set_p_sensor_cp(double x, double y, double z)
  {
    p_sensor_cp_ = KDL::Vector(x, y, z);
  }

  bool HybridImpedanceController::set_cmd(lwr_force_position_controllers::HybridImpedanceCommand::Request &req,\
					  lwr_force_position_controllers::HybridImpedanceCommand::Response &res)
  {
    Eigen::Vector3d desired_position;
    Eigen::Vector3d desired_attitude;

    // set requested position and attitude
    desired_position(0) = req.command.x;
    desired_position(1) = req.command.y;
    desired_attitude(0) = req.command.alpha;
    desired_attitude(1) = req.command.beta;
    desired_attitude(2) = req.command.gamma;

    // // set requested circular trajectory parameters
    // circle_trj_ = req.command.circle_trj;
    // circle_trj_frequency_ = req.command.frequency;
    // circle_trj_radius_ = req.command.radius;
    // circle_trj_center_x_ = req.command.center_x;
    // circle_trj_center_y_ = req.command.center_y;

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

    ///////////////////////////////
    p2p_traj_mutex_.lock();

    p2p_traj_duration_ = req.command.p2p_traj_duration;
    eval_point_to_point_traj_constants(desired_position, desired_attitude,\
				       p2p_traj_duration_);
    time_ = 0;

    p2p_traj_mutex_.unlock();
    //////////////////////////////
    
    /////////////////////////////
    force_traj_mutex_.lock();

    force_ref_duration_ = req.command.force_ref_duration;
    evaluate_force_reference_constants(req.command.forcez, force_ref_duration_);
    time_force_ = 0;    

    force_traj_mutex_.unlock();
    ////////////////////////////

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
    res.command.x = prev_pos_setpoint_(0);
    res.command.y = prev_pos_setpoint_(1);
    res.command.p2p_traj_duration = p2p_traj_duration_;

    // get attitude
    res.command.alpha = prev_att_setpoint_(0);
    res.command.beta = prev_att_setpoint_(1);
    res.command.gamma = prev_att_setpoint_(2);
    
    // get force
    res.command.forcez = prev_fz_setpoint_;
    res.command.force_ref_duration = force_ref_duration_;

    // get circle trajectory
    res.command.circle_trj = circle_trj_;
    res.command.frequency = circle_trj_frequency_;
    res.command.radius = circle_trj_radius_;
    res.command.center_x = circle_trj_center_x_;
    res.command.center_y = circle_trj_center_y_;

    return true;
  }

  void HybridImpedanceController::evaluate_force_reference_constants(double force_des, double duration)
  {
    force_ref_const_(0) = prev_fz_setpoint_;
    force_ref_const_(1) = FORCE_REF_COEFF_2 * (force_des - prev_fz_setpoint_) / pow(duration, 2);
    force_ref_const_(2) = FORCE_REF_COEFF_3 * (force_des - prev_fz_setpoint_) / pow(duration, 3);
    prev_fz_setpoint_ = force_des;
  }

  double HybridImpedanceController::eval_force_reference(const ros::Duration& period)
  {
    force_traj_mutex_.lock();

    time_force_ += period.toSec();
    if(time_force_ >= force_ref_duration_)
      time_force_ = force_ref_duration_;

    double value;
    value = force_ref_const_(2) * pow(time_force_, 3) + \
      force_ref_const_(1) * pow(time_force_, 2) + \
      force_ref_const_(0);

    force_traj_mutex_.unlock();
    
    return value;
  }

  void HybridImpedanceController::eval_current_point_to_point_traj(const ros::Duration& period,\
								   Eigen::VectorXd& x_des,\
								   Eigen::VectorXd& xdot_des,\
								   Eigen::VectorXd& xdotdot_des)
  {
    p2p_traj_mutex_.lock();

    time_ += period.toSec();

    if (time_ > p2p_traj_duration_)
      time_ = p2p_traj_duration_;

    for (int i=0; i<6; i++)
      {
	x_des(i) = p2p_trj_const_(0, i) + p2p_trj_const_(1, i) * pow(time_, 3) + \
	  p2p_trj_const_(2, i) * pow(time_, 4) + p2p_trj_const_(3, i) * pow(time_, 5);

	xdot_des(i) = 3 * p2p_trj_const_(1, i) * pow(time_, 2) + \
	  4 * p2p_trj_const_(2, i) * pow(time_, 3) + 5 * p2p_trj_const_(3, i) * pow(time_, 4);

	xdotdot_des(i) = 3 * 2 *  p2p_trj_const_(1, i) * time_ + \
	  4 * 3 * p2p_trj_const_(2, i) * pow(time_, 2) + 5 * 4 * p2p_trj_const_(3, i) * pow(time_, 3);
      }

    p2p_traj_mutex_.unlock();

  }

  void HybridImpedanceController::eval_point_to_point_traj_constants(Eigen::Vector3d& desired_position, \
								     Eigen::Vector3d& desired_attitude,
								     double duration)
  { 
    // evaluate common part of constants
    double constant_0, constant_1, constant_2;
    constant_0 = P2P_COEFF_3 / pow(duration, 3);
    constant_1 = P2P_COEFF_4 / pow(duration, 4);
    constant_2 = P2P_COEFF_5 / pow(duration, 5);

    // evaluate constants for x and y trajectories
    for (int i=0; i<2; i++)
      {
	double error = desired_position(i) - prev_pos_setpoint_(i);
	p2p_trj_const_(0, i) = prev_pos_setpoint_(i);
	p2p_trj_const_(1, i) = error * constant_0;
	p2p_trj_const_(2, i) = error * constant_1;
	p2p_trj_const_(3, i) = error * constant_2;
      }
    prev_pos_setpoint_ = desired_position;

    // evaluate constants alpha, beta and gamma trajectories
    double alpha_cmd, beta_cmd, gamma_cmd;
    KDL::Rotation::EulerZYZ(desired_attitude(0),
			    desired_attitude(1),
			    desired_attitude(2)).GetEulerZYZ(alpha_cmd,\
							     beta_cmd,
							     gamma_cmd);
    Eigen::Vector3d des_attitude_fixed;
    des_attitude_fixed << alpha_cmd, beta_cmd, gamma_cmd;
    for (int i=0; i<3; i++)
      {

	double error = angles::normalize_angle(des_attitude_fixed(i) - prev_att_setpoint_(i));
	p2p_trj_const_(0, i + 3) = prev_att_setpoint_(i);
	p2p_trj_const_(1, i + 3) = error * constant_0;
	p2p_trj_const_(2, i + 3) = error * constant_1;
	p2p_trj_const_(3, i + 3) = error * constant_2;
      }
    prev_att_setpoint_ = des_attitude_fixed;

  }

  // void HybridImpedanceController::eval_current_circular_traj(const ros::Duration& period)
  // {
  //   // evaluate the circular trajectory
  //   time_ = time_ + period.toSec();
  //   double f = circle_trj_frequency_;
  //   double omega = 2 * M_PI * f;
  //   double rho = circle_trj_radius_;
  //   double x_trj = circle_trj_center_x_ + rho * cos(omega * time_);
  //   double y_trj = circle_trj_center_y_ + rho * sin(omega * time_);
  //   double dx_trj = -omega * rho * sin(omega * time_);
  //   double dy_trj = omega * rho * cos(omega * time_);
  //   double ddx_trj = -omega * omega * rho * cos(omega * time_);
  //   double ddy_trj = -omega * omega * rho * sin(omega * time_);

  //   // set the position
  //   x_des_(0) = x_trj;
  //   x_des_(1) = y_trj;

  //   // set the velocity
  //   xdot_des_(0) = dx_trj;
  //   xdot_des_(1) = dy_trj;

  //   // set the acceleration
  //   xdotdot_des_(0) = ddx_trj;
  //   xdotdot_des_(1) = ddy_trj;
  // }

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
