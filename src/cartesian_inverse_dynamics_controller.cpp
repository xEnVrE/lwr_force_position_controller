#include <pluginlib/class_list_macros.h>
#include <utils/euler_kinematical_zyz.h>
#include <angles/angles.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/LU>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <lwr_force_position_controllers/cartesian_inverse_dynamics_controller.h>

#define DEFAULT_KP_IM_LINK4 0.001
#define DEFAULT_KP_IM_LINK5 3
#define DEFAULT_KD_IM_LINK4 10
#define DEFAULT_KD_IM_LINK5 10
// syntax:
//
// for jacobians x_J_y := Jacobian w.r.t reference point y expressed in basis x
// for analytical jacobians x_JA_y := analytical Jacobian w.r.t reference point y expressed in basis x
// for rotation matrices R_x_y := Rotation from basis y to basis x
// for wrenches x_wrench_y := Wrench w.r.t reference point y expressed in basis x 
// for vectors in general x_vector := vector expressed in basis x
// p_x_y := arm from x to y
// ee := reference point of interest (typically the tool tip)
// wrist := tip of the 7th link of the Kuka LWR
// T := euler kinematical matrix

namespace lwr_controllers {

  CartesianInverseDynamicsController::CartesianInverseDynamicsController() {}
  CartesianInverseDynamicsController::~CartesianInverseDynamicsController() {}

  bool CartesianInverseDynamicsController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

    // get use_simulation parameter from rosparam server
    ros::NodeHandle nh;
    nh.getParam("/use_simulation", use_simulation_);

    // extend kdl chain with end-effector
    extend_chain(n);

    // create two chain from vito_anchor to the link
    // specified by the parameter internal_motion_controlled_link
    std::string root_name, im_c_link_name;
    nh_.getParam("root_name", root_name);
    // nh_.getParam("internal_motion_controlled_link", im_c_link_name);
    kdl_tree_.getChain(root_name, "lwr_4_link", im_link4_chain_);
    kdl_tree_.getChain(root_name, "lwr_5_link", im_link5_chain_);

    // instantiate solvers
    // gravity_ is a member of KinematicChainControllerBase
    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    ee_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(extended_chain_));
    wrist_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    ee_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(extended_chain_));
    ee_jacobian_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(extended_chain_));
    im_link4_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(im_link4_chain_));
    im_link4_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(im_link4_chain_));
    im_link5_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(im_link5_chain_));
    im_link5_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(im_link5_chain_));
 
    // instantiate wrenches
    wrench_wrist_ = KDL::Wrench();
    base_wrench_wrist_ = KDL::Wrench();

    // instantiate state and its derivatives
    ws_x_ = Eigen::VectorXd(6);
    ws_xdot_ = Eigen::VectorXd(6);

    // instantiate analytical to geometric transformation matrices
    ws_TA_ = Eigen::MatrixXd::Zero(6,6);
    ws_TA_.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();
    ws_TA_dot_ = Eigen::MatrixXd::Zero(6,6);

    // instantiate analytical to geometric transformation matrices
    base_TA_im_link5_ = Eigen::MatrixXd::Zero(6,6);
    base_TA_im_link5_.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();

    // set default controller gains
    Kp_im_ = Eigen::Matrix<double, 6, 6>::Zero();
    Kd_im_ = Eigen::Matrix<double, 6, 6>::Zero();
    for(int i = 0; i < 3; i++)
      Kd_im_(i,i) = DEFAULT_KD_IM_LINK4;
    for(int i = 3; i < 6; i++)
      Kd_im_(i,i) = DEFAULT_KD_IM_LINK5;

    // set proportional action in the z direction only (for link4) (position)
    Kp_im_(2,2) = DEFAULT_KP_IM_LINK4;
    // set proportional action along z axis only (for link5) (attitude)
    Kp_im_(5,5) = DEFAULT_KP_IM_LINK5;

    // subscribe to force/torque sensor topic
    sub_force_ = n.subscribe(ft_sensor_topic_name_, 1, \
			     &CartesianInverseDynamicsController::force_torque_callback, this);

    return true;
  }

  void CartesianInverseDynamicsController::starting(const ros::Time& time) 
  {

    // get current robot configuration (q) for the 5th link
    KDL::JntArray q_im_link5;
    q_im_link5.resize(im_link5_chain_.getNrOfJoints());
    for(size_t i=0; i<im_link5_chain_.getNrOfJoints(); i++)
	q_im_link5(i) = joint_handles_[i].getPosition();

    // get current attitude for 5th link
    double alpha_im_link5, beta_im_link5, gamma_im_link5;
    KDL::Frame im_link5_fk_frame;
    im_link5_fk_solver_->JntToCart(q_im_link5, im_link5_fk_frame);
    im_link5_fk_frame.M.GetEulerZYZ(alpha_im_link5, beta_im_link5, gamma_im_link5);

    gamma_im_link5_initial_ = gamma_im_link5;
  }

  void CartesianInverseDynamicsController::update_fri_inertia_matrix(Eigen::MatrixXd& fri_B)
  {
    int n_joints = kdl_chain_.getNrOfJoints();
    for(int i = 0; i < n_joints; i++)
      for(int j = 0; j < n_joints; j++)
	fri_B(i,j) = inertia_matrix_handles_[i * n_joints + j].getPosition();
  }

  void CartesianInverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    //////////////////////////////////////////////////////////////////////////////////
    //
    // Robot configuration
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // get current robot configuration (q and q dot)
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      {
	joint_msr_states_.q(i) = joint_handles_[i].getPosition();
	joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
      }

    // get inertia matrix from FRI
    Eigen::MatrixXd fri_B (joint_handles_.size(), joint_handles_.size());
    update_fri_inertia_matrix(fri_B);
 
    // get the current configuration of the internal motion controlled links
    KDL::JntArray q_im_link4;
    KDL::JntArray q_im_link5;
    q_im_link4.resize(im_link4_chain_.getNrOfJoints());
    q_im_link5.resize(im_link5_chain_.getNrOfJoints());
    for(size_t i=0; i<im_link4_chain_.getNrOfJoints(); i++)
	q_im_link4(i) = joint_msr_states_.q(i);
    for(size_t i=0; i<im_link5_chain_.getNrOfJoints(); i++)
	q_im_link5(i) = joint_msr_states_.q(i);

    //
    //////////////////////////////////////////////////////////////////////////////////
    
    //////////////////////////////////////////////////////////////////////////////////
    //
    // Joint Space Inertia Matrix B and Coriolis term C * q dot
    // (solvers does not take into account anything past the wrist
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // evaluate the current B(q)
    KDL::JntSpaceInertiaMatrix B;
    B.resize(kdl_chain_.getNrOfJoints());
    dyn_param_solver_->JntToMass(joint_msr_states_.q, B);

    // evaluate the current C(q) * q dot
    KDL::JntArray C;
    C.resize(kdl_chain_.getNrOfJoints());
    dyn_param_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C);
    
    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // Geometric Jacobians
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // evaluate the current geometric jacobian base_J_ee
    KDL::Jacobian base_J_ee;
    base_J_ee.resize(kdl_chain_.getNrOfJoints());
    ee_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_ee);

    // evaluate the current geometric jacobian base_J_wrist
    KDL::Jacobian base_J_wrist;
    base_J_wrist.resize(kdl_chain_.getNrOfJoints());
    wrist_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_wrist);

    // evaluate the current geometric jacobian related to 
    // the internal motion controlled links (linear velocity of the third and
    // angular velocity of the fourth)
    KDL::Jacobian base_J_im_link4;
    KDL::Jacobian base_J_im_link5;
    Eigen::MatrixXd base_J_im = Eigen::MatrixXd::Zero(6,7);
    base_J_im_link4.resize(im_link4_chain_.getNrOfJoints());
    base_J_im_link5.resize(im_link5_chain_.getNrOfJoints());
    im_link4_jacobian_solver_->JntToJac(q_im_link4, base_J_im_link4);
    im_link5_jacobian_solver_->JntToJac(q_im_link5, base_J_im_link5);
    base_J_im.block(0, 0, 3, im_link4_chain_.getNrOfJoints()) = \
      base_J_im_link4.data.block(0, 0, 3, im_link4_chain_.getNrOfJoints());

    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // Forward Kinematics
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // end effector
    KDL::Frame ee_fk_frame;
    ee_fk_solver_->JntToCart(joint_msr_states_.q, ee_fk_frame);

    // internal motion controlled link
    KDL::Frame im_link4_fk_frame;
    KDL::Frame im_link5_fk_frame;
    im_link4_fk_solver_->JntToCart(q_im_link4, im_link4_fk_frame);
    im_link5_fk_solver_->JntToCart(q_im_link5, im_link5_fk_frame);

    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // Analytical Jacobian written w.r.t. the workspace frame ws_JA_ee
    // ws_JA_ee = ws_TA * ws_J_ee
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // get the current ZYZ attitude representation PHI from R_ws_base * ee_fk_frame.M
    double alpha, beta, gamma;
    R_ws_ee_ = R_ws_base_ * ee_fk_frame.M;
    R_ws_ee_.GetEulerZYZ(alpha, beta, gamma);
    
    // evaluate the transformation matrix between 
    // the geometric and analytical jacobian TA
    //
    // ws_TA = [eye(3), zeros(3);
    //        zeros(3), inv(T(PHI))]
    // where T is the Euler Kinematical Matrix
    //
    Eigen::Matrix3d ws_T;
    eul_kin_ZYZ(beta, alpha, ws_T);
    ws_TA_.block<3,3>(3,3) = ws_T.inverse();

    // evaluate ws_J_ee
    KDL::Jacobian ws_J_ee;
    ws_J_ee.resize(kdl_chain_.getNrOfJoints());
    KDL::changeBase(base_J_ee, R_ws_base_, ws_J_ee);

    Eigen::MatrixXd ws_JA_ee;
    ws_JA_ee = ws_TA_ * ws_J_ee.data;
    //
    //////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////////////
    //
    // Analytical Jacobian written w.r.t. the vito_anchor frame base_JA_im_link5
    // base_JA_im_link5 = base_TA_im_link5 * base_J_im_link5
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    double alpha_im_link5, beta_im_link5, gamma_im_link5;
    im_link5_fk_frame.M.GetEulerZYZ(alpha_im_link5, beta_im_link5, gamma_im_link5);
    
    // evaluate the transformation matrix between 
    // the geometric and analytical jacobian TA

    Eigen::Matrix3d base_T_im_link5;
    eul_kin_ZYZ(beta_im_link5, alpha_im_link5, base_T_im_link5);
    base_TA_im_link5_.block<3,3>(3,3) = base_T_im_link5.inverse();

    Eigen::MatrixXd base_JA_im_link5;
    base_JA_im_link5 = base_TA_im_link5_ * base_J_im_link5.data;

    // add the angular part of base_JA_im_link5 to base_J_im
    base_J_im.block(3, 0, 3, im_link5_chain_.getNrOfJoints()) = \
      base_JA_im_link5.block(3, 0, 3, im_link5_chain_.getNrOfJoints());

    //
    //////////////////////////////////////////////////////////////////////////////////

  
    //////////////////////////////////////////////////////////////////////////////////
    //
    // Kinetic pseudo-energy BA (Siciliano p. 297)
    // (i.e. Operational Space Inertia Matrix)
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // BA = inv(ws_JA_ee * B_inv * base_J_wrist')
    Eigen::MatrixXd BA;
    if(use_simulation_)
      // use kdl matrix for simulation
      BA = ws_JA_ee * B.data.inverse() * base_J_wrist.data.transpose();
    else
      // use kdl matrix for real scenario
      BA = ws_JA_ee * B.data.inverse() * base_J_wrist.data.transpose();

    BA = BA.inverse();

    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // Coriolis compensation in *Operational Space*
    // BA * dot(ws_JA_ee) * qdot
    //
    //////////////////////////////////////////////////////////////////////////////////
    //
    
    // evaluation of dot(ws_JA_ee) = d/dt{ws_TA} * ws_J_ee + ws_TA * d/dt{ws_J_ee}
    //
    // where d/dt{ws_TA} = [d/dt{eye(3)}, d/dt{zeros(3)}; 
    //                      d/dt{zeros(3)}, d/dt{inv(T(PHI))}]
    //                   = [zeros(3), zeros(3);
    //                      zeros(3), -inv(T) * d/dt{T} * int(T)]
    //
    // and d/dt{ws_J_ee} = [R_ws_base_, zeros(3);
    //                      zeros(3), R_ws_base_] * d/dt{base_J_ee}
    //
    
    // evaluate the derivative of the state using the analytical jacobian
    Eigen::Matrix3d ws_T_dot;
    ws_xdot_ = ws_JA_ee * joint_msr_states_.qdot.data;
    eul_kin_ZYZ_dot(beta, alpha, ws_xdot_(4), ws_xdot_(3), ws_T_dot);
    ws_TA_dot_.block<3,3>(3,3) = - ws_T.inverse() * ws_T_dot * ws_T.inverse();

    // evaluate the derivative of the jacobian base_J_ee
    KDL::JntArrayVel jnt_q_qdot;
    KDL::Jacobian ws_J_ee_dot;
    ws_J_ee_dot.resize(kdl_chain_.getNrOfJoints());
    jnt_q_qdot.q = joint_msr_states_.q;
    jnt_q_qdot.qdot = joint_msr_states_.qdot;
    ee_jacobian_dot_solver_->JntToJacDot(jnt_q_qdot, ws_J_ee_dot);
    
    // and project it in the workspace frame
    ws_J_ee_dot.changeBase(R_ws_base_);

    Eigen::MatrixXd ws_JA_ee_dot;
    ws_JA_ee_dot = ws_TA_dot_ * ws_J_ee.data + ws_TA_ * ws_J_ee_dot.data;
    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // project ft_sensor wrench in world frame
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    Eigen::Matrix<double, 6,1> base_F_wrist;
    base_wrench_wrist_ = ee_fk_frame.M * wrench_wrist_;
    tf::wrenchKDLToEigen(base_wrench_wrist_, base_F_wrist);

    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // evaluate dynamics inversion command TAU_FRI 
    //
    // inheriting controllers augment TAU_FRI by calling 
    // set_command(desired_acceleration)
    // so that TAU_FRI += command_filter * desired_accelration
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    if(use_simulation_)
      tau_fri_ = C.data + base_J_wrist.data.transpose() * \
    	(base_F_wrist - BA * ws_JA_ee_dot * joint_msr_states_.qdot.data);
    else
      // tau_fri_ = C.data + base_J_wrist.data.transpose() *		\
      // 	(base_F_wrist - BA * ws_JA_ee_dot * joint_msr_states_.qdot.data);
      tau_fri_ = C.data + base_J_wrist.data.transpose() *	\
	(- BA * ws_JA_ee_dot * joint_msr_states_.qdot.data);

    command_filter_ = base_J_wrist.data.transpose() * BA;
    
    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // internal motion handling 
    // (see A Unified Approach for Motion and Force Control
    // of Robot Manipulators: The Operational Space Formulation, Oussama Khatib
    // for details on the definition of a dynamically consistent generalized inverse)
    //
    //////////////////////////////////////////////////////////////////////////////////
    //  

    // evaluate a dynamically consistent generalized inverse
    Eigen::MatrixXd gen_inv;
    if(use_simulation_)
      gen_inv = B.data.inverse() * base_J_wrist.data.transpose() * \
	(base_J_wrist.data * B.data.inverse() * base_J_wrist.data.transpose()).inverse();
    else
      gen_inv = B.data.inverse() * base_J_wrist.data.transpose() * \
	(base_J_wrist.data * B.data.inverse() * base_J_wrist.data.transpose()).inverse();

    // evaluate the null space filter
    Eigen::MatrixXd ns_filter = Eigen::Matrix<double, 7, 7>::Identity() - \
      base_J_wrist.data.transpose() * gen_inv.transpose();

    // state and derivative of the state
    Eigen::VectorXd im_state(6);
    Eigen::VectorXd im_state_dot;

    im_state << im_link4_fk_frame.p.x(), im_link4_fk_frame.p.y(), im_link4_fk_frame.p.z(),\
      alpha_im_link5, beta_im_link5, gamma_im_link5;
    
    im_state_dot = base_J_im * joint_msr_states_.qdot.data;

    // control law
    // tau = null_space_filter * J_im' * (Kp * (x_des - x) - Kd * x_dot) 
    Eigen::VectorXd im_des_state = Eigen::VectorXd(6);

    // control strategy is to command an height offset between ee and link4
    // and command the attitude (gamma only) of the link5
    // in order to avoid joints limits
    double z_offset = 0.8;
    im_des_state << ee_fk_frame.p.x(), ee_fk_frame.p.y() + 0.6 , ee_fk_frame.p.z() + z_offset, \
      -M_PI / 2, M_PI / 2, 0;

    if(!(-M_PI / 2 < gamma_im_link5_initial_ && gamma_im_link5_initial_ < M_PI / 2))
	im_des_state(5) = M_PI;

    Eigen::VectorXd im_error = im_des_state - im_state;
    // normalize angular error between - M_PI and M_PI
    im_error(3) = angles::normalize_angle(im_error(3));
    im_error(4) = angles::normalize_angle(im_error(4));
    im_error(5) = angles::normalize_angle(im_error(5));

    // filter and add the command to TAU_FRI
    tau_fri_ += ns_filter * base_J_im.transpose() *\
      (Kp_im_ * im_error - Kd_im_ * im_state_dot);

    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // the following are possibly required by inheriting controllers
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // evaluate position vector between the origin of the workspace frame and the end-effector
    KDL::Vector p_ws_ee;
    p_ws_ee = R_ws_base_ * (ee_fk_frame.p - p_base_ws_);
    
    // evaluate the state
    ws_x_ << p_ws_ee(0), p_ws_ee(1), p_ws_ee(2), alpha, beta, gamma;

    // the derivative of the state 
    // was already evaluated in the previous sections

    //
    //////////////////////////////////////////////////////////////////////////////////

  }

  void CartesianInverseDynamicsController::set_command(Eigen::VectorXd& commanded_acceleration)
  {
    // augment tau_fri with the desired command specified by the inheriting controller
    tau_fri_ += command_filter_ * commanded_acceleration;

    // set joint efforts
    for(int i=0; i<kdl_chain_.getNrOfJoints(); i++)
      {
	joint_handles_[i].setCommand(tau_fri_(i));

	// required to exploit the JOINT IMPEDANCE MODE of the kuka manipulator
	joint_stiffness_handles_[i].setCommand(0);
	joint_damping_handles_[i].setCommand(0);
	joint_set_point_handles_[i].setCommand(joint_msr_states_.q(i));
      }
  }

  void CartesianInverseDynamicsController::load_calib_data(double& total_mass, KDL::Vector& p_sensor_tool_com)
  {
    std::string file_name = ros::package::getPath("lwr_force_position_controllers") +\
      "/config/ft_calib_data.yaml";
    YAML::Node ft_data_yaml = YAML::LoadFile(file_name);

    std::vector<double> p_sensor_tool_com_vec(6);
    // get data from the yaml file
    double tool_mass = ft_data_yaml["gripper_mass"].as<double>();
    p_sensor_tool_com_vec = ft_data_yaml["gripper_com_pose"].as<std::vector<double>>();

    // transform to KDL
    for (int i=0; i<3; i++)
	p_sensor_tool_com.data[i] = p_sensor_tool_com_vec[i];

    // compensate for additional items attached
    double mass_sensor_support = 0.194;
    double mass_sensor = 0.099;
    total_mass = tool_mass + mass_sensor_support + mass_sensor;
    
    p_sensor_tool_com.x(tool_mass * p_sensor_tool_com.x() / total_mass);
    p_sensor_tool_com.y(tool_mass * p_sensor_tool_com.y() / total_mass);
    p_sensor_tool_com.z((mass_sensor_support * 0.013 + mass_sensor * 0.0335 + \
			 tool_mass * (0.041 + p_sensor_tool_com.z())) / total_mass);

  }

  void CartesianInverseDynamicsController::extend_chain(ros::NodeHandle &n)
  {
    // extend the default chain with a fake segment in order to evaluate
    // dynamics (B and C), Jacobians, derivatives of jacobians and 
    // forward kinematics with respect to a given reference point
    // (typically the tool tip)
    // the reference point is initialized by the inheriting class with a call to set_p_wrist_ee
    KDL::Joint fake_joint = KDL::Joint();
    KDL::Frame frame(KDL::Rotation::Identity(), p_wrist_ee_);

    double total_mass;
    double fake_cylinder_radius  = 0.0475;
    double fake_cylinder_height = 0.31;
    KDL::Vector p_sensor_tool_com;
    load_calib_data(total_mass, p_sensor_tool_com);
        
    double fake_cyl_i_xx = 1.0 / 12.0 * total_mass * (3 * pow(fake_cylinder_radius, 2) + \
    						     pow(fake_cylinder_height, 2));
    double fake_cyl_i_zz = 1.0 / 2.0 * total_mass * pow(fake_cylinder_radius, 2);

    
    KDL::RotationalInertia rot_inertia(fake_cyl_i_xx, fake_cyl_i_xx, fake_cyl_i_zz);
    KDL::RigidBodyInertia inertia(total_mass, p_sensor_tool_com, rot_inertia);

    KDL::Segment fake_segment(fake_joint, frame, inertia);
    extended_chain_ = kdl_chain_;
    extended_chain_.addSegment(fake_segment);

  }
  
  void CartesianInverseDynamicsController::force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    KDL::Wrench wrench_wrist_topic;
    tf::wrenchMsgToKDL(msg->wrench, wrench_wrist_topic);
   
    // reverse the measured force so that wrench_wrist represents 
    // the force applied on the environment by the end-effector
    wrench_wrist_ = - wrench_wrist_topic;
  }

  void CartesianInverseDynamicsController::get_gains_im(double& kp_z, double& kp_gamma, double& kd_pos, double& kd_att)
  {
    kp_z = Kp_im_(2, 2);
    kp_gamma = Kp_im_(5, 5);
    kd_pos = Kd_im_(0, 0);
    kd_att = Kd_im_(3, 3);
  }

  void CartesianInverseDynamicsController::set_gains_im(double kp_z, double kp_gamma, double kd_pos, double kd_att)
  {
    Kp_im_(2,2) = kp_z;
    Kp_im_(5,5) = kp_gamma;
    for(int i = 0; i < 3; i++)
      Kd_im_(i,i) = kd_pos;
    for(int i = 3; i < 6; i++)
      Kd_im_(i,i) = kd_att;
  }

  void CartesianInverseDynamicsController::set_ft_sensor_topic_name(std::string topic)
  {
    ft_sensor_topic_name_ = topic;
  }

  void CartesianInverseDynamicsController::set_p_wrist_ee(double x, double y, double z)
  {
    p_wrist_ee_ = KDL::Vector(x, y, z);
  }
  
  void CartesianInverseDynamicsController::set_p_base_ws(double x, double y, double z)
  {
    p_base_ws_ = KDL::Vector(x, y, z);
  }

  void CartesianInverseDynamicsController::set_ws_base_angles(double alpha, double beta, double gamma)
  {
    R_ws_base_ = KDL::Rotation::EulerZYZ(alpha, beta, gamma);
  }
    
} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianInverseDynamicsController , controller_interface::ControllerBase)
