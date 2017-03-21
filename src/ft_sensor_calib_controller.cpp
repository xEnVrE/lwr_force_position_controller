#include <pluginlib/class_list_macros.h>
#include <lwr_force_position_controllers/ft_sensor_calib_controller.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames_io.hpp>
#include <math.h>
#include <utils/skew_pinv.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
//DEBUG DATA
#include <fstream>

#define G_FORCE 9.80665

namespace lwr_controllers {

  FtSensorCalibController::FtSensorCalibController() {}
  FtSensorCalibController::~FtSensorCalibController() {}

  bool FtSensorCalibController::init(hardware_interface::JointStateInterface *robot, ros::NodeHandle& nh)
  {
    KinematicChainControllerBase<hardware_interface::JointStateInterface>::init(robot, nh);
    
    // subscribe to raw ft sensor topic
    std::string ft_sensor_topic_name;
    nh.getParam("topic_name", ft_sensor_topic_name);
    sub_ft_sensor_ = nh.subscribe(ft_sensor_topic_name,
				  1,
				  &FtSensorCalibController::ft_raw_topic_callback, 
				  this);

    // get the calibration loop rate
    nh_.getParam("calibration_loop_rate", calibration_loop_rate_);

    // get publish rate from rosparam
    nh.getParam("publish_rate", publish_rate_);

    // get the trajectory duration from rosparam
    nh.getParam("p2p_traj_duration", p2p_traj_duration_);

    // reset ik solver
    ik_solver_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));
    
    // reset fk solver
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    
    // reset ft_calib
    ft_calib_.reset(new Calibration::FTCalib());

    // publisher to joint trajectory controller
    pub_joint_traj_ctl_ = nh.advertise<trajectory_msgs::JointTrajectory>("/lwr/joint_trajectory_controller/command", 1);

    // advertise services
    move_next_calib_pose_service_ = nh.advertiseService("move_next_calib_pose",\
							&FtSensorCalibController::move_next_calib_pose, this);
    move_home_pose_service_ = nh.advertiseService("move_home_pose",\
						  &FtSensorCalibController::move_home_pose, this);
    save_calib_data_service_ = nh.advertiseService("save_calib_data",\
						   &FtSensorCalibController::save_calib_data, this);
    start_compensation_service_ = nh.advertiseService("start_compensation",\
						   &FtSensorCalibController::start_compensation, this);
    do_estimation_step_service_ = nh.advertiseService("do_estimation_step",\
						      &FtSensorCalibController::do_estimation_step, this);
    start_autonomus_estimation_service_ = nh.advertiseService("start_autonomus_estimation",\
							      &FtSensorCalibController::start_autonomus_estimation, this);

    // advertise topic
    pub_ft_sensor_no_offset_ = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor_no_offset", 1);
    pub_ft_sensor_no_gravity_ = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor_no_gravity", 1);

    // load calibration poses
    //get_calibration_poses(nh);
    get_calibration_q(nh);

    // reset pose counter
    pose_counter_ = 0;

    // check if existing calibration data should be used
    bool recover;
    nh_.getParam("recover_existing_data", recover);
    if (recover)
      recover_existing_data();

    return true;
  }

  void FtSensorCalibController::starting(const ros::Time& time) 
  {
    // initialize time
    last_publish_time_ = time;
  }

  void FtSensorCalibController::update(const ros::Time& time, const ros::Duration& period)
  {
    if(!do_compensation_)
      return;

    // do offset compensation
    KDL::Wrench ft_wrench_no_offset;
    ft_wrench_no_offset = ft_wrench_raw_ - offset_kdl_;
    
    //get the current robot configuration
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();

    // evaluate forward kinematics
    KDL::Frame fk_frame;
    KDL::Rotation R_ft_base; //rotation from base frame to ft frame
    fk_solver_->JntToCart(joint_msr_states_.q, fk_frame);
    R_ft_base = fk_frame.M.Inverse();

    // compesante for tool weight
    // move the reference point of the weight from the com of the tool to the wrist
    KDL::Frame gravity_transformation(fk_frame.M.Inverse(),	\
				      p_sensor_tool_com_kdl_);
    // compensate for the weight of the tool
    KDL::Wrench ft_wrench_no_gravity;
    ft_wrench_no_gravity = ft_wrench_no_offset - gravity_transformation * base_tool_weight_com_;

    if(time > last_publish_time_ + ros::Duration(1.0 / publish_rate_))
      {
	//update next tick
	last_publish_time_ += ros::Duration(1.0 / publish_rate_);

	publish_data(ft_wrench_no_offset, pub_ft_sensor_no_offset_);
	publish_data(ft_wrench_no_gravity, pub_ft_sensor_no_gravity_);
      }
  }

  void FtSensorCalibController::ft_raw_topic_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    tf::wrenchMsgToKDL(msg->wrench, ft_wrench_raw_);
  }

  // void FtSensorCalibController::get_calibration_poses(ros::NodeHandle nh)
  // {
  //   nh.getParam("calib_number_q", number_of_poses_);
  //   for (int i=0; i<number_of_poses_; i++)
  //     {
  // 	std::vector<double> pose;
  // 	nh.getParam("calib_poses/pose" + std::to_string(i), pose);
  // 	ft_calib_poses_.push_back(KDL::Frame(KDL::Rotation::EulerZYX(pose.at(3), pose.at(4), pose.at(5)),
  // 					     KDL::Vector(pose.at(0), pose.at(1), pose.at(2))));
  //     }
  // }

  void FtSensorCalibController::get_calibration_q(ros::NodeHandle nh)
  {
    nh.getParam("calib_number_q", number_of_poses_);
    for (int i=0; i<number_of_poses_; i++)
      {
  	std::vector<double> q;
  	nh.getParam("calib_q/q" + std::to_string(i), q);
	ft_calib_q_.push_back(q);
      }
    nh.getParam("home_q", ft_calib_q_home_);
  }

  void FtSensorCalibController::send_joint_trajectory_msg(std::vector<double> q_des)
  {
    trajectory_msgs::JointTrajectory traj_msg;
    trajectory_msgs::JointTrajectoryPoint point;
    traj_msg.joint_names.push_back("lwr_a1_joint");
    traj_msg.joint_names.push_back("lwr_a2_joint");
    traj_msg.joint_names.push_back("lwr_e1_joint");
    traj_msg.joint_names.push_back("lwr_a3_joint");
    traj_msg.joint_names.push_back("lwr_a4_joint");
    traj_msg.joint_names.push_back("lwr_a5_joint");
    traj_msg.joint_names.push_back("lwr_a6_joint");
    for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
      point.positions.push_back(q_des.at(i));
    point.time_from_start = ros::Duration(p2p_traj_duration_);
    traj_msg.points.push_back(point);
    pub_joint_traj_ctl_.publish(traj_msg);
  }

  bool FtSensorCalibController::move_home_pose(std_srvs::Empty::Request& req,\
						     std_srvs::Empty::Response& res)
  {
    send_joint_trajectory_msg(ft_calib_q_home_);
    return true;
  }

  bool FtSensorCalibController::move_next_calib_pose(std_srvs::Empty::Request& req,\
						     std_srvs::Empty::Response& res)
  {
    if (number_of_poses_ <= pose_counter_)
      {
	std::cout << "Manual pose " << pose_counter_ << std::endl;
	return true;
      }

    std::cout << "Sending pose " << pose_counter_ << " to joint_trajectory_controller" << std::endl;

    send_joint_trajectory_msg(ft_calib_q_.at(pose_counter_));

    return true;
  }

  void FtSensorCalibController::add_measurement(KDL::Vector gravity_ft, KDL::Wrench ft_raw_avg)
  {
    // transform data so that it can be used in the FTCalib class
    std::string frame_id = "ft_frame";

    geometry_msgs::Vector3Stamped gravity_msg;
    geometry_msgs::WrenchStamped ft_avg_msg;

    gravity_msg.header.frame_id = frame_id;
    gravity_msg.vector.x = gravity_ft.x();
    gravity_msg.vector.y = gravity_ft.y();
    gravity_msg.vector.z = gravity_ft.z();
    
    ft_avg_msg.header.frame_id = frame_id;
    ft_avg_msg.wrench.force.x = ft_raw_avg.force.x();
    ft_avg_msg.wrench.force.y = ft_raw_avg.force.y();
    ft_avg_msg.wrench.force.z = ft_raw_avg.force.z();
    ft_avg_msg.wrench.torque.x = ft_raw_avg.torque.x();
    ft_avg_msg.wrench.torque.y = ft_raw_avg.torque.y();
    ft_avg_msg.wrench.torque.z = ft_raw_avg.torque.z();

    // add measurement
    ft_calib_->addMeasurement(gravity_msg, ft_avg_msg);
  }

  bool FtSensorCalibController::start_autonomus_estimation(std_srvs::Empty::Request& req,\
							   std_srvs::Empty::Response& res)
  {
    ros::Duration wait = ros::Duration(p2p_traj_duration_ + 4);
    for(int i = 0; i<number_of_poses_; i++)
      {
	// move robot
	std::cout << "Sending pose " << i << " to joint_trajectory_controller" << std::endl;
	send_joint_trajectory_msg(ft_calib_q_.at(i));

	// wait for trajectory end
	wait.sleep();

	// do a step of the estimation proces
	std::cout << "Estimation n.: " << i << std::endl; 
	estimation_step();

	// update counter
	pose_counter_ = i;

	//save estimation data
	save_debug_data();
      }

    std::cout << "Finish estimation process"<< std::endl; 
    return true;
  }

  void FtSensorCalibController::estimation_step()
  {
    ros::Rate loop_rate(calibration_loop_rate_);
    int number_measurements = 100;
    KDL::Wrench ft_raw_avg;

    // average 101 measurements from the ft sensor
    ft_raw_avg = ft_wrench_raw_;
    for (int i=0; i<number_measurements; i++)
      {
    	ft_raw_avg = ft_raw_avg + ft_wrench_raw_;

	loop_rate.sleep();
      }

    ft_raw_avg = - ft_raw_avg / double(number_measurements + 1);

    //get the current robot configuration
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();

    // evaluate forward kinematics
    KDL::Frame fk_frame;
    KDL::Rotation R_ft_base; //rotation from base frame to ft frame
    fk_solver_->JntToCart(joint_msr_states_.q, fk_frame);
    R_ft_base = fk_frame.M.Inverse();
    
    // evaluate the gravity as if it was measured by an IMU
    // whose reference frame is aligned with vito_anchor
    KDL::Vector gravity(0, 0, -G_FORCE);

    // rotate the gravity in ft frame
    KDL::Vector gravity_ft;
    gravity_ft = R_ft_base * gravity;

    geometry_msgs::Vector3Stamped gravity_msg;
    geometry_msgs::WrenchStamped ft_avg_msg;

    // add measurement to the filter
    add_measurement(gravity_ft, ft_raw_avg);

    // save data to allow data recovery if needed
    save_calib_meas(gravity_ft, ft_raw_avg, pose_counter_, joint_msr_states_.q);

    // get current estimation
    Eigen::VectorXd ft_calib = ft_calib_->getCalib();

    tool_mass_ = ft_calib(0);

    p_sensor_tool_com_(0) = ft_calib(1) / tool_mass_;
    p_sensor_tool_com_(1) = ft_calib(2) / tool_mass_;
    p_sensor_tool_com_(2) = ft_calib(3) / tool_mass_;

    ft_offset_force_(0) = -ft_calib(4);
    ft_offset_force_(1) = -ft_calib(5);
    ft_offset_force_(2) = -ft_calib(6);

    ft_offset_torque_(0) = -ft_calib(7);
    ft_offset_torque_(1) = -ft_calib(8);
    ft_offset_torque_(2) = -ft_calib(9);
  }

  bool FtSensorCalibController::do_estimation_step(std_srvs::Empty::Request& req,\
						   std_srvs::Empty::Response& res)
  {
    // do a step of the estimation proces
    estimation_step();
    
    // update pose_counter
    pose_counter_++;

    // save estimation data
    save_debug_data();

    return true;
  }

  void FtSensorCalibController::recover_existing_data()
  {
    std::string file_name = ros::package::getPath("lwr_force_position_controllers") +\
      "/config/ft_calib_meas.yaml";
    YAML::Node ft_data_yaml = YAML::LoadFile(file_name);

    int number = ft_data_yaml["calib_meas_number"].as<int>();

    KDL::Vector gravity_ft;
    KDL::Wrench ft_wrench_avg;
    std::vector<double> gravity_vec(3);
    std::vector<double> ft_force_avg(3);
    std::vector<double> ft_torque_avg(3);
    
    for (int i=0; i < number; i++)
      {
	gravity_vec = ft_data_yaml["calib_meas"]["pose" + std::to_string(i)]["gravity"].as<std::vector<double>>();
	ft_force_avg = ft_data_yaml["calib_meas"]["pose" + std::to_string(i)]["force_avg"].as<std::vector<double>>();
	ft_torque_avg = ft_data_yaml["calib_meas"]["pose" + std::to_string(i)]["torque_avg"].as<std::vector<double>>();

	for (int i=0; i<3; i++)
	  {
	    gravity_ft.data[i] = gravity_vec[i];
	    ft_wrench_avg.force.data[i] = ft_force_avg[i];
	    ft_wrench_avg.torque.data[i] = ft_torque_avg[i]; 
	  }

	add_measurement(gravity_ft, ft_wrench_avg);
	pose_counter_++;
      }

    std::cout << "Recovered calibration data for " << number << " poses" << std::endl;
  }

  void FtSensorCalibController::save_calib_meas(KDL::Vector gravity, KDL::Wrench ft_wrench_avg, int index,\
						KDL::JntArray q_kdl)
  {
    std::string file_name = ros::package::getPath("lwr_force_position_controllers") + \
      "/config/ft_calib_meas.yaml";
    YAML::Node ft_data_yaml = YAML::LoadFile(file_name);

    std::vector<double> gravity_vec(3);
    std::vector<double> ft_force_avg(3);
    std::vector<double> ft_torque_avg(3);
    std::vector<double> q(7);
    for (int i=0; i<3; i++)
      {
	gravity_vec[i] = gravity.data[i];
	ft_force_avg[i] = ft_wrench_avg.force.data[i];
	ft_torque_avg[i] = ft_wrench_avg.torque.data[i];
      }
    for (int i=0; i<7; i++)
      q[i] = q_kdl(i);

    ft_data_yaml["calib_meas_number"] = index + 1;
    ft_data_yaml["calib_meas"]["pose" + std::to_string(index)]["gravity"] = gravity_vec;
    ft_data_yaml["calib_meas"]["pose" + std::to_string(index)]["force_avg"] = ft_force_avg;
    ft_data_yaml["calib_meas"]["pose" + std::to_string(index)]["torque_avg"] = ft_torque_avg;
    ft_data_yaml["calib_meas"]["pose" + std::to_string(index)]["q"] = q;

    std::ofstream yaml_out(file_name);
    yaml_out << ft_data_yaml;
  }

  bool FtSensorCalibController::save_calib_data(std_srvs::Empty::Request& req,\
						std_srvs::Empty::Response& res)
  {
    std::string file_name = ros::package::getPath("lwr_force_position_controllers") +\
      "/config/ft_calib_data.yaml";
    YAML::Node ft_data_yaml = YAML::LoadFile(file_name);

    // convert data to standard types
    std::string frame_id = "lwr_7_link";
    std::vector<double> ft_offset(6);
    std::vector<double> p_sensor_tool_com(6);

    for (int i=0; i<3; i++)
      {
	ft_offset[i] = ft_offset_force_(i);
	ft_offset[i + 3] = ft_offset_torque_(i);
	p_sensor_tool_com[i] = p_sensor_tool_com_(i);
	p_sensor_tool_com[i + 3] = 0;
      }
    
    // populate yaml using field names
    // required by the package ros-indigo-gravity-compensation
    ft_data_yaml["bias"] = ft_offset;
    ft_data_yaml["gripper_com_frame_id"] = frame_id;
    ft_data_yaml["gripper_com_pose"] = p_sensor_tool_com;
    ft_data_yaml["gripper_mass"] = tool_mass_;

    // save yaml file
    std::ofstream yaml_out(file_name);
    yaml_out << ft_data_yaml;

    return true;
  }

  void FtSensorCalibController::save_debug_data()
  {
    // save current estimation
    std::string filename = ros::package::getPath("lwr_force_position_controllers") + \
      "/config/data_estimation.txt";

    std::ofstream data_file;

    data_file.open (filename, std::ios::out | std::ios::app);

    data_file << pose_counter_ << ", " << tool_mass_ << ", " \
	      << p_sensor_tool_com_(0) << ", " << p_sensor_tool_com_(1) << ", " << p_sensor_tool_com_(2) << ", " \
	      << ft_offset_force_(0) << ", " << ft_offset_force_(1) << ", " << ft_offset_force_(2) << ", " \
	      << ft_offset_torque_(0) << ", " << ft_offset_torque_(1) << ", " << ft_offset_torque_(2) << std::endl;

    data_file.close();

    // print the current estimation
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "Current calibration estimate:" << std::endl;
    std::cout << std::endl << std::endl;

    std::cout << "Mass: " << tool_mass_ << std::endl << std::endl;

    std::cout << "Tool CoM (in ft sensor frame):" << std::endl;
    std::cout << "[" << p_sensor_tool_com_(0) << ", " << p_sensor_tool_com_(1) << ", " << p_sensor_tool_com_(2) << "]";
    std::cout << std::endl << std::endl;

    std::cout << "FT offset: " << std::endl;
    std::cout << "[" << ft_offset_force_(0) << ", " << ft_offset_force_(1) << ", " << ft_offset_force_(2) << ", ";
    std::cout << ft_offset_torque_(0) << ", " << ft_offset_torque_(1) << ", " << ft_offset_torque_(2) << "]";
    std::cout << std::endl << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl << std::endl << std::endl;

  }

  bool FtSensorCalibController::load_calib_data()
  {
    std::string file_name = ros::package::getPath("lwr_force_position_controllers") +\
      "/config/ft_calib_data.yaml";
    YAML::Node ft_data_yaml = YAML::LoadFile(file_name);

    std::vector<double> p_sensor_tool_com(6);
    std::vector<double> offset(6);

    // get data from the yaml file
    tool_mass_ = ft_data_yaml["gripper_mass"].as<double>();
    p_sensor_tool_com = ft_data_yaml["gripper_com_pose"].as<std::vector<double>>();
    offset = ft_data_yaml["bias"].as<std::vector<double>>();

    // evaluate end-effector weight
    base_tool_weight_com_ = KDL::Wrench(KDL::Vector(0, 0, -tool_mass_ * G_FORCE),\
					KDL::Vector::Zero());

    // transform to KDL
    for (int i=0; i<3; i++)
      {
	p_sensor_tool_com_kdl_.data[i] = p_sensor_tool_com[i];
	offset_kdl_.force.data[i] = offset[i];
	offset_kdl_.torque.data[i] = offset[i + 3];
      }

    return true;
  }

  bool FtSensorCalibController::start_compensation(std_srvs::Empty::Request& req,\
						   std_srvs::Empty::Response& res)
  {
    // load calibration data from yaml file
    load_calib_data();

    // enable offset and gravity compensation
    do_compensation_ = true;

    return true;
  }

  void FtSensorCalibController::publish_data(KDL::Wrench wrench, ros::Publisher& pub)
  {
    // create the message
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.x = wrench.force.x(); 
    wrench_msg.wrench.force.y = wrench.force.y(); 
    wrench_msg.wrench.force.z = wrench.force.z(); 
    wrench_msg.wrench.torque.x = wrench.torque.x(); 
    wrench_msg.wrench.torque.y = wrench.torque.y(); 
    wrench_msg.wrench.torque.z = wrench.torque.z();
    
    // publish the message
    pub.publish(wrench_msg);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::FtSensorCalibController, controller_interface::ControllerBase)

  // bool FtSensorCalibController::move_next_calib_pose(std_srvs::Empty::Request& req,\
  // 						     std_srvs::Empty::Response& res)
  // {
  //   if (number_of_poses_ == pose_counter_)
  //     return true;

  //   KDL::JntArray q_des;
  //   KDL::Frame x_des;
  //   KDL::Vector des_pose;
  //   KDL::Rotation des_attitude;
      
  //   q_des.resize(kdl_chain_.getNrOfJoints());

  //   //get the current robot configuration
  //   for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
  //     joint_msr_states_.q(i) = joint_handles_[i].getPosition();

  //   std::cout << "pose " << pose_counter_ << std::endl;

  //   // get the desired frame
  //   x_des = ft_calib_poses_.at(pose_counter_);

  //   // evaluate q_des = ik(x_des)
  //   ik_solver_->CartToJnt(joint_msr_states_.q, x_des, q_des);

  //   // normalize q_des between - M_PI and M_PI
  //   for(int i=0; i<kdl_chain_.getNrOfJoints(); i++) 
  //     q_des(i) =  angles::normalize_angle(q_des(i));

  //   // verify that q_des respect joint position limits 
  //   for(int i=0; i<kdl_chain_.getNrOfJoints(); i++) 
  //     if((q_des(i) <=  joint_limits_.min(i)) || (q_des(i) >= joint_limits_.max(i)))
  // 	{
  // 	  std::cout<< "[cart_pos]Solution found unfeasible. Solution will bed modified for joint " \
  // 		   << i << std::endl;
  // 	  double correction = 10 * M_PI / 180;
  // 	  if(q_des(i) <= joint_limits_.min(i))
  // 	    q_des(i) += correction;
  // 	  else
  // 	    q_des(i) -= correction;
  // 	}

  //   ft_calib_q_.push_back(q_des);
	
  //   trajectory_msgs::JointTrajectory traj_msg;
  //   trajectory_msgs::JointTrajectoryPoint point;
  //   traj_msg.joint_names.push_back("lwr_a1_joint");
  //   traj_msg.joint_names.push_back("lwr_a2_joint");
  //   traj_msg.joint_names.push_back("lwr_e1_joint");
  //   traj_msg.joint_names.push_back("lwr_a3_joint");
  //   traj_msg.joint_names.push_back("lwr_a4_joint");
  //   traj_msg.joint_names.push_back("lwr_a5_joint");
  //   traj_msg.joint_names.push_back("lwr_a6_joint");
  //   for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  //     point.positions.push_back(q_des(i));
  //   point.time_from_start = ros::Duration(1.0);
  //   traj_msg.points.push_back(point);
  //   pub_joint_traj_ctl_.publish(traj_msg);

  //   pose_counter_++;

  //   return true;
  // }

  // bool FtSensorCalibController::save_ft_data(std_srvs::Empty::Request& req,\
  // 					      std_srvs::Empty::Response& res)
  // {
  //   std::string file_name = ros::package::getPath("lwr_force_position_controllers") +\
  //     "/config/ft_calib_data.yaml";
    
  //   YAML::Node ft_data_yaml = YAML::LoadFile(file_name);

  //   Eigen::VectorXd q;
  //   std::vector<double> q_std;
  //   for(int i=0; i<ft_calib_q_.size(); i++)
  //     {
  // 	q = ft_calib_q_.at(i).data;
  // 	q_std.resize(q.size());
  // 	Eigen::VectorXd::Map(&q_std[0], q.size()) = q;
  // 	ft_data_yaml["lwr"]["ft_sensor_calib_controller"]["calib_q"]["q" + std::to_string(i)] = q_std;
  //     }

  //   std::ofstream yaml_out(file_name);
  //   yaml_out << ft_data_yaml;
  // }
