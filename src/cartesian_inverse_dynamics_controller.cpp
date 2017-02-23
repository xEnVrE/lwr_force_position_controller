#include <pluginlib/class_list_macros.h>
#include <utils/euler_kinematical_rpy.h>
#include <angles/angles.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/LU>
#include <lwr_force_position_controllers/cartesian_inverse_dynamics_controller.h>

#define DEFAULT_KP_IM 30
#define DEFAULT_KD_IM 30
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
    nh.getParam("use_simulation", use_simulation_);

    // advertise CartesianInverseCommand service
    set_cmd_service_ = n.advertiseService("set_cartesian_inverse_command",\
					  &CartesianInverseDynamicsController::set_cmd, this);
    get_cmd_service_ = n.advertiseService("get_cartesian_inverse_command",\
					  &CartesianInverseDynamicsController::get_cmd, this);

    // extend the default chain with a fake segment in order to evaluate
    // Jacobians, derivatives of jacobians and forward kinematics with respect to a given reference point
    // (typicallly the tool tip)
    // the reference point is initialized by the inheriting class with a call to set_p_wrist_ee
    KDL::Joint fake_joint = KDL::Joint();
    KDL::Frame frame(KDL::Rotation::Identity(), p_wrist_ee_);
    KDL::Segment fake_segment(fake_joint, frame);
    extended_chain_ = kdl_chain_;
    extended_chain_.addSegment(fake_segment);

    // create a new chain from vito_anchor to the link
    // specified by the parameter internal_motion_controlled_link
    std::string root_name, im_c_link_name;
    nh_.getParam("root_name", root_name);
    nh_.getParam("internal_motion_controlled_link", im_c_link_name);
    kdl_tree_.getChain(root_name, im_c_link_name, im_chain_);

    // instantiate solvers
    // gravity_ is a member of KinematicChainControllerBase
    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    ee_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(extended_chain_));
    wrist_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    ee_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(extended_chain_));
    ee_jacobian_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(extended_chain_));
    im_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(im_chain_));
    im_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(im_chain_));
    
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

    // set default controller gains
    Kp_im_ = Eigen::Matrix<double, 3, 3>::Zero(); 
    Kd_im_ = Eigen::Matrix<double, 3, 3>::Identity() * DEFAULT_KD_IM;
    // set proportional action in the z direction only
    Kp_im_(2,2) = DEFAULT_KP_IM;


    // subscribe to force/torque sensor topic
    sub_force_ = n.subscribe("/lwr/ft_sensor_controller/ft_sensor_nog", 1,\
			     &CartesianInverseDynamicsController::force_torque_callback, this);

    return true;
  }

  void CartesianInverseDynamicsController::starting(const ros::Time& time) {}

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
 
    // get the current configuration of the internal motion controlled link
    KDL::JntArray q_im;
    q_im.resize(im_chain_.getNrOfJoints());
    for(size_t i=0; i<im_chain_.getNrOfJoints(); i++)
	q_im(i) = joint_msr_states_.q(i);

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
    // the internal motion controlled link (linear velocity only)
    KDL::Jacobian base_J_im;
    Eigen::MatrixXd base_J_im_linear = Eigen::MatrixXd::Zero(3,7);
    base_J_im.resize(im_chain_.getNrOfJoints());
    im_jacobian_solver_->JntToJac(q_im, base_J_im);
    base_J_im_linear.block(0, 0, 3, im_chain_.getNrOfJoints()) = \
      base_J_im.data.block(0, 0, 3, im_chain_.getNrOfJoints());

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
    KDL::Frame im_fk_frame;
    im_fk_solver_->JntToCart(q_im, im_fk_frame);

    //
    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////
    //
    // Analytical Jacobian written w.r.t. the workspace frame ws_JA_ee
    // ws_JA_ee = ws_TA * ws_J_ee
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // get the current RPY attitude representation PHI from R_ws_base * ee_fk_frame.M
    double yaw, pitch, roll;
    R_ws_ee_ = R_ws_base_ * ee_fk_frame.M;
    R_ws_ee_.GetEulerZYX(yaw, pitch, roll);
    
    // evaluate the transformation matrix between 
    // the geometric and analytical jacobian TA
    //
    // ws_TA = [eye(3), zeros(3);
    //        zeros(3), inv(T(PHI))]
    // where T is the Euler Kinematical Matrix
    //
    Eigen::Matrix3d ws_T;
    eul_kin_RPY(pitch, yaw, ws_T);
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
    // Kinetic pseudo-energy BA (Siciliano p. 297)
    // (i.e. Operational Space Inertia Matrix)
    //
    //////////////////////////////////////////////////////////////////////////////////
    //

    // BA = inv(ws_JA_ee * B_inv * base_J_wrist')
    Eigen::MatrixXd BA;
    if(use_simulation_)
      BA = ws_JA_ee * B.data.inverse() * base_J_wrist.data.transpose();
    else
      BA = ws_JA_ee * fri_B.inverse() * base_J_wrist.data.transpose();

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
    eul_kin_RPY_dot(pitch, yaw, ws_xdot_(4), ws_xdot_(3), ws_T_dot);
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
      tau_fri_ = base_J_wrist.data.transpose() * \
	(base_F_wrist - BA * ws_JA_ee_dot * joint_msr_states_.qdot.data);

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
      gen_inv = fri_B.inverse() * base_J_wrist.data.transpose() * \
	(base_J_wrist.data * fri_B.inverse() * base_J_wrist.data.transpose()).inverse();

    // evaluate the null space filter
    Eigen::MatrixXd ns_filter = Eigen::Matrix<double, 7, 7>::Identity() - \
      base_J_wrist.data.transpose() * gen_inv.transpose();

    // state and derivative of the state
    Eigen::Matrix<double, 3, 1> im_link_state;
    Eigen::VectorXd im_link_state_dot;
    tf::vectorKDLToEigen(im_fk_frame.p, im_link_state);
    im_link_state_dot = base_J_im_linear * joint_msr_states_.qdot.data;

    // control law
    // is very *RAW* and similar to the Kuka LWR Cartesian Impedance mode of operation
    // tau = null_space_filter * J_im' * (Kp * (x_des - x) - Kd * x_dot)
    // 
    Eigen::VectorXd im_link_des_state = Eigen::VectorXd(3);

    // control strategy is to command an height offset between ee and im link
    double offset = 0.5;
    im_link_des_state << 0, 0 , ee_fk_frame.p.z() + offset;

    // filter and add the command to TAU_FRI
    tau_fri_ += ns_filter * base_J_im_linear.transpose() * \
      (Kp_im_ * (im_link_des_state - im_link_state) - Kd_im_ * im_link_state_dot);

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
    ws_x_ << p_ws_ee(0), p_ws_ee(1), p_ws_ee(2), yaw, pitch, roll;

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

  void CartesianInverseDynamicsController::force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    KDL::Wrench wrench_wrist_topic;
    tf::wrenchMsgToKDL(msg->wrench, wrench_wrist_topic);
   
    // reverse the measured force so that wrench_wrist represents 
    // the force applied on the environment by the end-effector
    wrench_wrist_ = - wrench_wrist_topic;
  }

  bool CartesianInverseDynamicsController::set_cmd(lwr_force_position_controllers::CartesianInverseCommand::Request &req,\
						   lwr_force_position_controllers::CartesianInverseCommand::Response &res)
  {
    // set gains
    if (req.command.kp_im != -1)
      {
	Kp_im_ = Eigen::Matrix<double, 3, 3>::Zero();
	Kp_im_(2,2) = req.command.kp_im;
      }
    if (req.command.kd_im != -1)
      Kd_im_ = Eigen::Matrix<double, 3, 3>::Identity() * req.command.kd_im;

    return true;
  }

  bool CartesianInverseDynamicsController::get_cmd(lwr_force_position_controllers::CartesianInverseCommand::Request &req,\
						   lwr_force_position_controllers::CartesianInverseCommand::Response &res)
  {
    // get gains
    res.command.kp_im = Kp_im_(2, 2);
    res.command.kd_im = Kd_im_(0, 0);
    
    return true;
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
    R_ws_base_ = KDL::Rotation::EulerZYX(alpha, beta, gamma);
  }
    
} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianInverseDynamicsController , controller_interface::ControllerBase)
