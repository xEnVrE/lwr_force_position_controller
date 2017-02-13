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

namespace lwr_controllers {

  HybridImpedanceController::HybridImpedanceController() {}

  HybridImpedanceController::~HybridImpedanceController() {}
  
  bool HybridImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    // get parameters required by 
    // CartesianInverseDynamicsController::init(robot, n) from rosparam server 
    get_parameters(n);
    // calibrate ft sensor measurement
    calibrate_ft_sensor(n);

    // this should be called after get_parameters and calibrate_ft_sensor ONLY
    CartesianInverseDynamicsController::init(robot, n);
    
    // instantiate the desired trajectory
    x_des_ = Eigen::VectorXd(6);
    xdot_des_ = Eigen::VectorXd(6);
    xdotdot_des_ = Eigen::VectorXd(6);
      
    // advertise SetHybridImpedanceCommand service
    cmd_service_ = n.advertiseService("set_hybrid_impedance_command", &HybridImpedanceController::set_cmd, this);
    
    // instantiate default controller gains
    Kp_ = Eigen::Matrix<double, 6, 6>::Identity() * DEFAULT_KP;
    Kd_ = Eigen::Matrix<double, 6, 6>::Identity() * DEFAULT_KD; 
    km_f_ = DEFAULT_KM_F;
    kd_f_ = DEFAULT_KD_F;

    // evaluate default trajectory
    double yaw_des, pitch_des, roll_des;
    KDL::Rotation R_des = KDL::Rotation::RotY(M_PI);
    R_des.GetEulerZYX(yaw_des, pitch_des, roll_des);
    x_des_ << 0, 0, 0.1, yaw_des, pitch_des, roll_des;
    xdot_des_ << 0, 0, 0, 0, 0, 0;
    xdotdot_des_ << 0, 0, 0, 0, 0, 0;
    fz_des_ = 0;
    circle_trj_ = false;
    circle_trj_frequency_ = DEFAULT_CIRCLE_FREQ;
    circle_trj_radius_ = DEFAULT_CIRCLE_RADIUS;
    circle_trj_center_x_ = DEFAULT_CIRCLE_CENTER_X;
    circle_trj_center_y_ = DEFAULT_CIRCLE_CENTER_Y;
    time_ = 0;

    pub_force_ = n.advertise<geometry_msgs::WrenchStamped>("ft_sensor_nog", 1000);
    pub_force_des_ = n.advertise<geometry_msgs::WrenchStamped>("force_des", 1000);
    pub_state_ = n.advertise<geometry_msgs::WrenchStamped>("state", 1000);
    pub_dstate_ = n.advertise<geometry_msgs::WrenchStamped>("dstate", 1000);
    pub_x_des_ = n.advertise<geometry_msgs::WrenchStamped>("x_des", 1000);
    pub_xdot_des_ = n.advertise<geometry_msgs::WrenchStamped>("xdot_des", 1000);
    pub_xdotdot_des_ = n.advertise<geometry_msgs::WrenchStamped>("xdotdot_des", 1000);


    return true;
  }

  void HybridImpedanceController::starting(const ros::Time& time)
  {
    CartesianInverseDynamicsController::starting(time);
  }

  void HybridImpedanceController::update(const ros::Time& time, const ros::Duration& period)
  {
    // evaluate inverse dynamics
    CartesianInverseDynamicsController::update(time, period);

    // the base class evaluates the following quantities every update
    // ws_x_: distance and attitude between the workspace and the tool tip (workspace basis)
    // ws_xdot_: derivative of ws_x_ (workspace basis)
    // base_wrench_wrist_: forces and torques with the reference point on the wrist (world_base basis)

    // move the reference point from the wrist to the tool tip and project in workspace basis
    KDL::Frame force_transformation(R_ws_base_, R_ws_ee_ * (-p_wrist_ee_));
    KDL::Wrench ws_F_ee;
    ws_F_ee = force_transformation * base_wrench_wrist_;

    if (circle_trj_)
      set_circular_traj(period);

    // evaluate state error
    Eigen::VectorXd err_x = x_des_ - ws_x_;

    // normalize angular error between - M_PI and M_PI
    err_x(3) = angles::normalize_angle(err_x(3));
    err_x(4) = angles::normalize_angle(err_x(4));
    err_x(5) = angles::normalize_angle(err_x(5));

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
    acc_cmd(2) = km_f_ * (-kd_f_ / km_f_ * ws_xdot_(2) + (fz_des_ - ws_F_ee.force.z()));

    set_command(acc_cmd);

    publish_data(pub_state_, ws_x_);
    publish_data(pub_dstate_, ws_xdot_);
    publish_data(pub_x_des_, x_des_);
    publish_data(pub_xdot_des_, xdot_des_);
    publish_data(pub_xdotdot_des_, xdotdot_des_);

    // publish wrench
    publish_data(pub_force_, ws_F_ee);
    publish_data(pub_force_des_, KDL::Wrench(KDL::Vector(0, 0, fz_des_),\
					       KDL::Vector(0, 0, 0)));
  }

  void HybridImpedanceController::calibrate_ft_sensor(ros::NodeHandle &n)
  {
    // get initial force torque sensor measurement from rosparam server
    // wrench is expressed in ee base
    std::vector<double> wrench;
    n.getParam("initial_ft_sensor_wrench", wrench);

    // set ee_calibration_wrench_ in the super class
    set_initial_ft_sensor_wrench(KDL::Wrench(KDL::Vector(wrench.at(0), wrench.at(1), wrench.at(2)), \
					     KDL::Vector(wrench.at(3), wrench.at(4), wrench.at(5))));
  }

  void HybridImpedanceController::get_parameters(ros::NodeHandle &n)
  {
    // vector from the wrist to the tool tip (projected in lwr_link_7 frame)
    std::vector<double> p_wrist_ee;

    // vector from vito_anchor to the workspace (projected in world frame)
    std::vector<double> p_base_ws;

    // attitude of the workspace frame w.r.t. vito_anchor frame
    std::vector<double> ws_base_angles;

    // vector from the wrist to the CoM of the tool (projected in lwr_link_7 frame)
    std::vector<double> p_wrist_ee_com;

    double tool_mass;

    // get parameters form rosparameter server
    n.getParam("p_wrist_ee", p_wrist_ee);
    n.getParam("p_base_ws", p_base_ws);
    n.getParam("ws_base_angles", ws_base_angles);
    n.getParam("p_wrist_ee_com", p_wrist_ee_com);
    n.getParam("tool_mass", tool_mass);

    // set internal members
    set_p_wrist_ee(p_wrist_ee.at(0), p_wrist_ee.at(1), p_wrist_ee.at(2));
    set_p_base_ws(p_base_ws.at(0), p_base_ws.at(1), p_base_ws.at(2));
    set_ws_base_angles(ws_base_angles.at(0), ws_base_angles.at(1), ws_base_angles.at(2));
    set_tool_weight(tool_mass);
    set_p_wrist_ee_com(p_wrist_ee_com.at(0), p_wrist_ee_com.at(1), p_wrist_ee_com.at(2));
  }

  bool HybridImpedanceController::set_cmd(lwr_force_position_controllers::SetHybridImpedanceCommand::Request &req,\
					  lwr_force_position_controllers::SetHybridImpedanceCommand::Response &res)
  {
    // set the desired position requested by the user
    x_des_(0) = req.command.x;
    x_des_(1) = req.command.y;
    // reset the derivatives 
    xdot_des_(0) = 0;
    xdot_des_(1) = 0;
    xdotdot_des_(0) = 0;
    xdotdot_des_(1) = 0;

    // set the desired force
    fz_des_ = req.command.z;
    
    // set circle trajectory
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

  void HybridImpedanceController::set_circular_traj(const ros::Duration& period)
  {
    // evaluate circle trajectory
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

    // set the desired position requested by the trajectory
    x_des_(0) = x_trj;
    x_des_(1) = y_trj;

    // set the desired velocity requested by the trajectory
    xdot_des_(0) = dx_trj;
    xdot_des_(1) = dy_trj;

    // set the desired acceleration requested by the trajectory
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
