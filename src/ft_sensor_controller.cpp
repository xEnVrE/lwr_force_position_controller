#include <pluginlib/class_list_macros.h>
#include <lwr_force_position_controllers/ft_sensor_controller.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames_io.hpp>
#include <math.h>
#include <utils/skew_pinv.h>
#include <ros/package.h>

namespace lwr_controllers {

  FtSensorController::FtSensorController() {}
  FtSensorController::~FtSensorController() {}

  bool FtSensorController::init(hardware_interface::JointStateInterface *robot, ros::NodeHandle& nh)
  {
    KinematicChainControllerBase<hardware_interface::JointStateInterface>::init(robot, nh);
    
    // instantiate forward kinematic solver
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    
    // subscribe to ft sensor topic
    std::string ft_sensor_topic_name;
    nh.getParam("topic_name", ft_sensor_topic_name);
    sub_ft_sensor_ = nh.subscribe(ft_sensor_topic_name, 1, &FtSensorController::ft_sensor_topic_callback, this);

    // advertise topic
    pub_ft_sensor_ = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor_alt", 1000);
    pub_ft_sensor_nog_ = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor_nog", 1000);

    // get publish rate from rosparam
    nh.getParam("publish_rate", publish_rate_);

    // get the ft sensor offset from rosparam and set ft_sensor_offset_
    std::vector<double> ft_sensor_offset;
    nh.getParam("ft_sensor_offset", ft_sensor_offset);
    set_wrench(ft_sensor_offset, ft_sensor_offset_);

    // get p_wrist_toolcom and set p_wrist_toolcom_
    // p_wrist_toolcom_ = vector from wrist center to the com of the tool
    std::vector<double> p;
    nh.getParam("p_wrist_toolcom", p);
    set_vector(p, p_wrist_toolcom_);

    // get tool mass and set base_tool_weight_com_
    // base_tool_weight_com_ = weight due to tool mass with reference point on its com
    // projected in world frame
    std::vector<double> weight;
    nh.getParam("base_tool_weight_com", weight);
    set_wrench(weight, base_tool_weight_com_);

    // advertise SensorCtlInitService service
    sensor_ctl_init_service_ = nh.advertiseService("sensor_ctl_init",\
						   &FtSensorController::set_sensor_initial_conditions,\
						   this);
    return true;
  }

  void FtSensorController::starting(const ros::Time& time) 
  {
    // initialize time
    last_publish_time_ = time;
  }

  void FtSensorController::update(const ros::Time& time, const ros::Duration& period)
  {
    KDL::Wrench wrench, wrench_nog;

    // sensor offsets its measure
    // so it has to be corrected like this
    wrench = ft_wrench_raw_;              //<--- from topic callback
    wrench += ft_sensor_offset_;          //<--- set in init()
    
    // get current robot configuration
    KDL::JntArray joint_position;
    joint_position.resize(kdl_chain_.getNrOfJoints());
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
      joint_position(i) = joint_handles_[i].getPosition();

    // evaluate forward kinematics
    KDL::Frame fk_frame;
    fk_solver_->JntToCart(joint_position, fk_frame);

    // compesante for tool weight
    // move the reference point of the weight from the com of the tool to the wrist
    KDL::Frame gravity_transformation(KDL::Rotation::Identity(),\
				      fk_frame.M * p_wrist_toolcom_);
    // compensate for the weight of the tool
    wrench_nog = fk_frame.M * wrench - gravity_transformation * base_tool_weight_com_;

    // the real would output measurements in end-effector frame
    wrench_nog = fk_frame.M.Inverse() * wrench_nog;

    if(time > last_publish_time_ + ros::Duration(1.0 / publish_rate_))
      {
	//update next tick
	last_publish_time_ += ros::Duration(1.0 / publish_rate_);

	publish_data(wrench, pub_ft_sensor_);
	publish_data(wrench_nog, pub_ft_sensor_nog_);
      }
  }

  void FtSensorController::publish_data(KDL::Wrench wrench, ros::Publisher& pub)
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

  void FtSensorController::ft_sensor_topic_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    tf::wrenchMsgToKDL(msg->wrench, ft_wrench_raw_);
  }

  bool FtSensorController::set_sensor_initial_conditions(std_srvs::Empty::Request& req,	\
							 std_srvs::Empty::Response& res)
  {
    // set ft_sensor_offset_
    ft_sensor_offset_ = ft_wrench_raw_;

    // evaluate end-effector weight
    base_tool_weight_com_ = KDL::Wrench(KDL::Vector(0, 0, -ft_sensor_offset_.force.Norm()),\
					KDL::Vector::Zero());
    
    // evaluate the coordinates of the tool com
    // torque = p_wrist_toolcom x force = -force x p_wrist_toolcom
    //        = -skew(force) * p_wrist_toolcom

    // p_wrist_toolcom = - pinv(skew(force)) * torque
    Eigen::Matrix<double, 3, 1> force;
    Eigen::Matrix<double, 3, 1> torque;
    Eigen::Matrix3d pinv;
    
    tf::vectorKDLToEigen(ft_sensor_offset_.force, force);
    tf::vectorKDLToEigen(ft_sensor_offset_.torque, torque);

    skew_pinv(force, pinv);
    tf::vectorEigenToKDL(-pinv * torque, p_wrist_toolcom_);

    // save to file    
    write_vector_to_yaml("ft_sensor_offset", ft_sensor_offset_);
    write_vector_to_yaml("base_tool_weight_com", base_tool_weight_com_);
    write_vector_to_yaml("p_wrist_toolcom", p_wrist_toolcom_);

    return true;
  }

  void FtSensorController::write_vector_to_yaml(std::string field, KDL::Wrench wrench)
  {
    // save vector to controllers.yaml
    std::string file_name = ros::package::getPath("lwr_force_position_controllers") + "/config/controllers.yaml";

    YAML::Node controllers_yaml = YAML::LoadFile(file_name);

    std::vector<double> value(6);
    value.at(0) = wrench.force.x();
    value.at(1) = wrench.force.y();
    value.at(2) = wrench.force.z();
    value.at(3) = wrench.torque.x();
    value.at(4) = wrench.torque.y();
    value.at(5) = wrench.torque.z();

    controllers_yaml["lwr"]["ft_sensor_controller"][field] = value;
    std::ofstream fout(file_name); 
    fout << controllers_yaml;
  }

  void FtSensorController::write_vector_to_yaml(std::string field, KDL::Vector vector)
  {
    // save vector to controllers.yaml
    std::string file_name = ros::package::getPath("lwr_force_position_controllers") + "/config/controllers.yaml";
    
    YAML::Node controllers_yaml = YAML::LoadFile(file_name);

    std::vector<double> value(3);
    value.at(0) = vector.x();
    value.at(1) = vector.y();
    value.at(2) = vector.z();

    controllers_yaml["lwr"]["ft_sensor_controller"][field] = value;
    std::ofstream fout(file_name); 
    fout << controllers_yaml;
  }

  void FtSensorController::set_wrench(std::vector<double>& value, KDL::Wrench& wrench)
  {
    wrench = KDL::Wrench(KDL::Vector(value.at(0), value.at(1), value.at(2)), \
			 KDL::Vector(value.at(3), value.at(4), value.at(5)));
  }

  void FtSensorController::set_vector(std::vector<double>& p, KDL::Vector& vector)
  {
    vector = KDL::Vector(p.at(0), p.at(1), p.at(2));
  }
    
} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::FtSensorController, controller_interface::ControllerBase)
