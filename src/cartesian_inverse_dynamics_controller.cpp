#include <pluginlib/class_list_macros.h>
#include <utils/euler_kinematical_rpy.h>
#include <angles/angles.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/LU>
#include <lwr_force_position_controllers/cartesian_inverse_dynamics_controller.h>

// syntax:
//
// for jacobians x_J_y := Jacobian w.r.t reference point y expressed in basis x
// for analytical jacobians x_JA_y := analytical Jacobian w.r.t reference point y expressed in basis x
// for rotation matrices R_x_y := Rotation from basis y to basis x
// ee := reference point of interest (typically the tool tip)
// T := euler kinematical matrix

namespace lwr_controllers {

  CartesianInverseDynamicsController::CartesianInverseDynamicsController() {}
  CartesianInverseDynamicsController::~CartesianInverseDynamicsController() {}

  bool CartesianInverseDynamicsController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

    // Extend the default chain with a fake segment in order to evaluate
    // Jacobians, derivatives of jacobians and forward kinematics with respect to a given reference point
    // (typicallly the tool tip)
    KDL::Joint fake_joint = KDL::Joint();
    KDL::Frame frame(KDL::Rotation::Identity(), p_wrist_ee_);
    KDL::Segment fake_segment(fake_joint, frame);
    extended_chain_ = kdl_chain_;
    extended_chain_.addSegment(fake_segment);

    // instantiate solvers
    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    ee_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(extended_chain_));
    wrist_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    ee_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(extended_chain_));
    ee_jacobian_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(extended_chain_));
    
    // resize initial robot configuration q0
    q0_.resize(kdl_chain_.getNrOfJoints());
    // resize Coriolis vector
    C_.resize(kdl_chain_.getNrOfJoints());
    // resize joint space inertia matrix
    B_.resize(kdl_chain_.getNrOfJoints());
    // resize jacobians
    base_J_wrist_.resize(kdl_chain_.getNrOfJoints());
    // resize wrenches
    wrench_wrist_ = KDL::Wrench();
    base_wrench_wrist_ = KDL::Wrench();

    // instantiate state and its derivatives (used in inheriting class)
    ws_x_ = Eigen::VectorXd(6);
    ws_xdot_ = Eigen::VectorXd(6);

    // instantiate analytical to geometric transformation matrices
    ws_TA_ = Eigen::MatrixXd::Zero(6,6);
    ws_TA_.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();
    ws_TA_dot_ = Eigen::MatrixXd::Zero(6,6);

    // subscribe to force/torque sensor topic
    sub_force_ = n.subscribe("/lwr/ft_sensor_controller/ft_sensor_nog"\
			     , 1, &CartesianInverseDynamicsController::force_torque_callback, this);

    return true;
  }

  void CartesianInverseDynamicsController::starting(const ros::Time& time)
  {
    // save the current robot configuration
    // required to compensate internal motion of the robot
    for(size_t i=0; i<joint_handles_.size(); i++) 
      q0_(i) = joint_handles_[i].getPosition();

  }

  void CartesianInverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    // get current configuration (position and velocity)
    for(size_t i=0; i<joint_handles_.size(); i++) {
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();
      joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }

    // solvers

    // evaluate the current B(q) (B is evaluated without taking into account anything past the wrist)
    dyn_param_solver_->JntToMass(joint_msr_states_.q, B_);

    // evaluate the current C(q) * q_dot (C is evaluated without taking into account anything past the wrist)
    dyn_param_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);

    // evaluate the current geometric jacobian base_J_ee
    KDL::Jacobian base_J_ee;
    base_J_ee.resize(kdl_chain_.getNrOfJoints());
    ee_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_ee);

    // evaluate the current geometric jacobian base_J_wrist
    wrist_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_wrist_);

    // evaluate the current homogeneous transformation from the base 
    // to the point of interest (typically the tool tip) ee_fk_frame_
    ee_fk_solver_->JntToCart(joint_msr_states_.q, ee_fk_frame_);


    /////////////////////////////////////////////////////////////////////////
    //
    // evaluate the analytical jacobian written w.r.t. the intermediate frame ws_JA_ee_
    // ws_JA_ee_ = ws_TA * ws_J_ee
    //
    /////////////////////////////////////////////////////////////////////////
    //

    // get the current RPY attitude representation PHI from R_ws_base * ee_fk_frame_.M
    double yaw, pitch, roll;
    R_ws_ee_ = R_ws_base_ * ee_fk_frame_.M;
    R_ws_ee_.GetEulerZYX(yaw, pitch, roll);
    
    // evaluate the transformation matrix between the geometric and analytical jacobian TA
    // ws_TA = [eye(3), zeros(3);
    //        zeros(3), inv(T(PHI))]
    // where T is the Euler Kinematical Matrix
    Eigen::Matrix3d ws_T;
    eul_kin_RPY(pitch, yaw, ws_T);
    ws_TA_.block<3,3>(3,3) = ws_T.inverse();

    // evaluate ws_J_ee
    KDL::Jacobian ws_J_ee;
    ws_J_ee.resize(kdl_chain_.getNrOfJoints());
    KDL::changeBase(base_J_ee, R_ws_base_, ws_J_ee);

    ws_JA_ee_ = ws_TA_ * ws_J_ee.data;
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate the kinetic pseudo-energy BA (Siciliano p. 297)
    //
    ////////////////////////////////////////////////////////////////////////

    // BA = inv(ws_JA_ee_ * B_inv * base_J_wrist')
    BA_ = ws_JA_ee_ * B_.data.inverse() * base_J_wrist_.data.transpose();
    BA_ = BA_.inverse();

    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate part of the Coriolis compensation in operational space
    // BA * dot(ws_JA_ee_) * qdot
    //
    ////////////////////////////////////////////////////////////////////////
    //
    
    // evaluation of dot(ws_JA_ee_) = d/dt{ws_TA} * ws_J_ee + ws_TA * d/dt{ws_J_ee}
    //
    // where d/dt{ws_TA} = [d/dt{eye(3)}, d/dt{zeros(3)}; 
    //                      d/dt{zeros(3)}, d/dt{inv(T(PHI))}]
    //                   = [zeros(3), zeros(3);
    //                      zeros(3), -inv(T) * d/dt{T} * int(T)]
    //
    // and d/dt{ws_J_ee} = [R_ws_base_, zeros(3);
    //                      zeros(3), R_ws_base_] * d/dt{base_J_ee}
    
    // evaluate the derivative of the state using the analytical jacobian
    Eigen::Matrix3d ws_T_dot;
    ws_xdot_ = ws_JA_ee_ * joint_msr_states_.qdot.data;
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

    ws_JA_ee_dot_ = ws_TA_dot_ * ws_J_ee.data + ws_TA_ * ws_J_ee_dot.data;
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate force and torque compensation
    // compensate for the weight of the tool
    // base_wrench_wrist = [R_base_wrist_, zeros(3);
    //                      zeros(3), R_base_wrist_] * wrench_wrist
    //
    ////////////////////////////////////////////////////////////////////////
    //
    
    // project wrench onto world frame
    base_wrench_wrist_ = ee_fk_frame_.M * wrench_wrist_;
    tf::wrenchKDLToEigen(base_wrench_wrist_, base_F_wrist_);
    
    //
    ////////////////////////////////////////////////////////////////////////

    // evaluate position vector between the origin of the workspace frame and the end-effector
    KDL::Vector p_ws_ee;
    p_ws_ee = R_ws_base_ * (ee_fk_frame_.p - p_base_ws_);
    
    // evaluate the state for the inheriting controller
    ws_x_ << p_ws_ee(0), p_ws_ee(1), p_ws_ee(2), yaw, pitch, roll;
  }

  void CartesianInverseDynamicsController::set_command(Eigen::VectorXd& commanded_acceleration)
  {
    // evaluate tau_FRI
    double k_p_null = 1;
    double k_d_null = 1;
    Eigen::VectorXd tau_fri;
    tau_fri = C_.data +\
      base_J_wrist_.data.transpose() *\
      (base_F_wrist_ + BA_ * (commanded_acceleration - ws_JA_ee_dot_ * joint_msr_states_.qdot.data)) +\
      (Eigen::Matrix<double, 7, 7>::Identity() - base_J_wrist_.data.transpose() * BA_ * ws_JA_ee_ * B_.data.inverse())*
      (k_p_null * (q0_.data - joint_msr_states_.q.data) - k_d_null * joint_msr_states_.qdot.data);
      

    // set joint efforts
    for(int i=0; i<kdl_chain_.getNrOfJoints(); i++)
      {
	joint_handles_[i].setCommand(tau_fri(i));

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

    // // compensate for initial ft sensor calibration
    // wrench_wrist_topic += ee_calibration_wrench_;
   
    // reverse the measured force so that wrench_wrist represents 
    // the force applied on the environment by the end-effector
    wrench_wrist_ = - wrench_wrist_topic;
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
