#include <pluginlib/class_list_macros.h>

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <angles/angles.h>
#include <eigen_conversions/eigen_kdl.h>
#include <math.h>

#include <lwr_force_position_controllers/cartesian_position_controller.h>

#define DEFAULT_KP 30
#define DEFAULT_KD 30

namespace lwr_controllers {

  CartesianPositionController::CartesianPositionController() {}

  CartesianPositionController::~CartesianPositionController() {}

  bool CartesianPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

    // get use_simulation parameter from rosparam server
    ros::NodeHandle nh;
    nh.getParam("use_simulation", use_simulation_);

    // get the p_wrist_ee arm from the rosparam server
    std::vector<double> p_wrist_ee;
    n.getParam("p_wrist_ee", p_wrist_ee);
    p_wrist_ee_ = KDL::Vector(p_wrist_ee.at(0), p_wrist_ee.at(1), p_wrist_ee.at(2));

    // Extend the default chain with a fake segment in order to evaluate
    // Jacobians and forward kinematics with respect to a given reference point
    // (typicallly the tool tip)
    KDL::Joint fake_joint = KDL::Joint();
    KDL::Frame frame(KDL::Rotation::Identity(), p_wrist_ee_);
    KDL::Segment fake_segment(fake_joint, frame);
    extended_chain_ = kdl_chain_;
    extended_chain_.addSegment(fake_segment);

    // instantiate solvers
    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    ik_solver_.reset(new KDL::ChainIkSolverPos_LMA(extended_chain_));
    ee_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(extended_chain_));
    if (use_simulation_)
      {
	jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
      }

    // set the default gains
    kp_ = DEFAULT_KP;
    kd_ = DEFAULT_KD;

    // resize and set desired quantities
    q_des_.resize(kdl_chain_.getNrOfJoints());
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      {
	//defaults to the current configuration
	q_des_(i) = joint_handles_[i].getPosition();
      }

    // advertise CartesianPositionCommand service
    set_cmd_service_ = n.advertiseService("set_cartesian_position_command", \
					  &CartesianPositionController::set_cmd, this); 

    get_cmd_service_ = n.advertiseService("get_cartesian_position_command", \
					  &CartesianPositionController::get_cmd, this); 

    if(use_simulation_)
      {
	// subscribe to force/torque sensor topic
	// (simulation only since it is required to compensate for the mass tool,
	// the real kuka compensate for mass tool internally)
	sub_force_ = n.subscribe("/lwr/ft_sensor_controller/ft_sensor_alt", 1,\
				 &CartesianPositionController::ft_sensor_callback, this);
      }
	
    use_inverse_dynamics_controller_ = false;

    return true;
  }

  void CartesianPositionController::starting(const ros::Time& time)
  {
    // instantiate desired quantities
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      {
	//defaults to the current configuration
	q_des_(i) = joint_handles_[i].getPosition();
      }
  }

  void CartesianPositionController::update(const ros::Time& time, const ros::Duration& period)
  {
    // get current configuration (position and velocity)
    for(size_t i=0; i<joint_handles_.size(); i++)
      {
	joint_msr_states_.q(i) = joint_handles_[i].getPosition();
	joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
      }
  
    // compute control law
    KDL::JntArray tau_cmd;
    tau_cmd.resize(kdl_chain_.getNrOfJoints());
    for(size_t i=0; i<joint_handles_.size(); i++)
      tau_cmd(i) = kp_ * (q_des_(i) - joint_msr_states_.q(i)) - kd_ * joint_msr_states_.qdot(i);

    // ONLY for inverse dynamics strategy
    KDL::JntSpaceInertiaMatrix B;
    KDL::JntArray C;
    B.resize(kdl_chain_.getNrOfJoints());
    C.resize(kdl_chain_.getNrOfJoints());
    if(use_inverse_dynamics_controller_)
      {
	// evaluate inertia matrix and coriolis effort required to invert the dynamics
	dyn_param_solver_->JntToMass(joint_msr_states_.q, B);
	dyn_param_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C);
      }
    
    KDL::Jacobian J_;
    Eigen::Matrix<double, 6,1> base_F_wrist_;
    if (use_simulation_)
      {
	////////////////////////////////////////////////////////////////////////////////
	// evaluate the wrench applied to the wrist from the force/torque sensor
	// and project it in the world frame (base)
	// this is required to compensate for additional masses attached past the wrist
	// (simulation only, the real kuka compensate for mass tool internally)
	////////////////////////////////////////////////////////////////////////////////
	//
	
	// evaluate the current geometric jacobian J(q)
	J_.resize(kdl_chain_.getNrOfJoints());
	jacobian_solver_->JntToJac(joint_msr_states_.q, J_);
	
	// evaluate the forward kinematics required to project the wrench
	KDL::Frame fk_frame;
	KDL::Wrench base_wrench_wrist;
	fk_solver_->JntToCart(joint_msr_states_.q, fk_frame);
	base_wrench_wrist = fk_frame.M * wrench_wrist_;
	
	// write down onto an eigen vector
	tf::wrenchKDLToEigen(base_wrench_wrist, base_F_wrist_);      
	//
	///////////////////////////////////////////////////////////////////////////////
      }

    // ONLY for inverse dynamics strategy
    if(use_inverse_dynamics_controller_)
      {
	// evaluate B * tau_cmd
	KDL::JntArray B_tau_cmd;
	B_tau_cmd.resize(kdl_chain_.getNrOfJoints());
	if (use_simulation_)
	  // use J * base_F_wrist as a way to compensate for the mass of the tool (simulation only)
	  B_tau_cmd.data = B.data * tau_cmd.data + C.data + J_.data.transpose() * base_F_wrist_;
	else
	  B_tau_cmd.data = B.data * tau_cmd.data + C.data;

	// set joint efforts
	for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
	  {
	    joint_handles_[i].setCommand(B_tau_cmd(i));
	    
	    // required to exploit the JOINT IMPEDANCE MODE of the kuka manipulator
	    joint_stiffness_handles_[i].setCommand(0);
	    joint_damping_handles_[i].setCommand(0);
	    joint_set_point_handles_[i].setCommand(joint_msr_states_.q(i));
	  }
      }

    // ONLY for PD strategy
    else
      {
	if (use_simulation_)
	  // use J * base_F_wrist as a way to compensate for the mass of the tool (simulation only)
	  tau_cmd.data = tau_cmd.data + J_.data.transpose() * base_F_wrist_;

	// set joint efforts
	for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
	  {
	    joint_handles_[i].setCommand(tau_cmd(i));
	    
	    // required to exploit the JOINT IMPEDANCE MODE of the kuka manipulator
	    joint_stiffness_handles_[i].setCommand(0);
	    joint_damping_handles_[i].setCommand(0);
	    joint_set_point_handles_[i].setCommand(joint_msr_states_.q(i));
	  }
      }
  }
  
  void CartesianPositionController::ft_sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {    
    KDL::Wrench wrench_wrist_topic;
    tf::wrenchMsgToKDL(msg->wrench, wrench_wrist_topic);
    
    // reverse the measured force so that wrench_wrist represents 
    // the force applied on the environment by the end-effector
    wrench_wrist_ = - wrench_wrist_topic;
  }

  bool CartesianPositionController::set_cmd(lwr_force_position_controllers::CartesianPositionCommand::Request &req,\
					  lwr_force_position_controllers::CartesianPositionCommand::Response &res)
  {
    KDL::Vector des_pose = KDL::Vector::Zero();
    KDL::Rotation des_attitude = KDL::Rotation::Identity();

    // set the desired position requested by the user
    des_pose.x(req.command.x);
    des_pose.y(req.command.y);
    des_pose.z(req.command.z);

    // set the desired attitude requested by the user
    des_attitude = KDL::Rotation::EulerZYX(req.command.yaw,\
					    req.command.pitch,\
					    req.command.roll);

    // set the desired gains requested by the user
    // TEMPORARY: -1 means that the user requested the last gain set
    if (req.command.kp != -1)
      kp_ = req.command.kp;
    if (req.command.kd != -1)
      kd_ = req.command.kd;

    // set desired controller strategy
    use_inverse_dynamics_controller_ =  res.command.use_inverse_dynamics_controller;

    // evaluate the new desired configuration 
    evaluate_q_des(des_pose, des_attitude);

    return true;
  }

  bool CartesianPositionController::get_cmd(lwr_force_position_controllers::CartesianPositionCommand::Request &req,\
					    lwr_force_position_controllers::CartesianPositionCommand::Response &res)
  {
    KDL::Frame ee_fk_frame;
    double yaw, pitch, roll;
    ee_fk_solver_->JntToCart(q_des_, ee_fk_frame);
    ee_fk_frame.M.GetEulerZYX(yaw, pitch, roll);
    
    // get desired position 
    res.command.x = ee_fk_frame.p.x();
    res.command.y = ee_fk_frame.p.y();
    res.command.z = ee_fk_frame.p.z();

    // get desired attitude
    res.command.yaw = yaw;
    res.command.pitch = pitch;
    res.command.roll = roll;

    // get desired gain
    res.command.kp = kp_;
    res.command.kd = kd_;

    res.command.use_inverse_dynamics_controller = use_inverse_dynamics_controller_;
    return true;
  }

  void CartesianPositionController::evaluate_q_des(KDL::Vector& des_pose,\
						   KDL::Rotation& des_attitude)
  {    
    // evaluate the new desired configuration q_des

    KDL::Frame x_des;
    KDL::JntArray q_des;
    q_des.resize(kdl_chain_.getNrOfJoints());

    // get the current robot configuration
    for(size_t i=0; i<joint_handles_.size(); i++)
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();

    // compose the desired frame
    x_des = KDL::Frame(des_attitude, des_pose);

    // evaluate q_des = ik(x_des)
    ik_solver_->CartToJnt(joint_msr_states_.q, x_des, q_des);

    // normalize q_des between - M_PI and M_PI
    for(int i=0; i<joint_handles_.size(); i++) 
      q_des(i) =  angles::normalize_angle(q_des(i));

    // verify that q_des respect joint position limits 
    print_joint_array(q_des);
    for(int i=0; i<joint_handles_.size(); i++) 
      if((q_des(i) <  joint_limits_.min(i)) || (q_des(i) >  joint_limits_.max(i)))
	{
	  std::cout<<"limits achieves"<<std::endl;
	  return;
	}
    q_des_ = q_des;
    
    return;
  }

  void CartesianPositionController::print_joint_array(KDL::JntArray& array)
  {
    std::cout << std::fixed;
    std::cout << std::setprecision(3);
	
    std::cout<<"joint \t q_des \t q_min\t q_max \t"<<std::endl;
    std::cout<<"*******************************"<<std::endl;

    for(int i=0; i<joint_handles_.size(); i++)
      std::cout << i << "\t" << array(i) << "\t" << joint_limits_.min(i) << "\t" << joint_limits_.max(i) << std::endl;
      
    std::cout<<"*******************************"<<std::endl;
  }

}// namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianPositionController, controller_interface::ControllerBase)
