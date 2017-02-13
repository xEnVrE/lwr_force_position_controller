#include <pluginlib/class_list_macros.h>
#include <lwr_force_position_controllers/ft_sensor_controller.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/frames_io.hpp>
#include <math.h>

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
    std::vector<double> wrench;
    nh.getParam("ft_sensor_offset", wrench);
    set_sensor_offset(wrench);

    // get p_wrist_toolcom and set p_wrist_toolcom_
    // p_wrist_toolcom_ = vector from wrist center to the com of the tool
    std::vector<double> p;
    nh.getParam("p_wrist_toolcom", p);
    set_p_wrist_toolcom(p);

    // get tool mass and set base_tool_weight_com_
    // base_tool_weight_com_ = weight due to tool mass with reference point on its com
    // projected in world frame
    double mass;
    nh.getParam("tool_mass", mass);
    set_base_tool_weight_com(mass);

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

    // transform wrench from end effector frame to world frame
    wrench = fk_frame.M * wrench;

    // compesante for tool weight
    // move the reference point of the weight from the com of the tool to the wrist
    KDL::Frame gravity_transformation(KDL::Rotation::Identity(),\
				      fk_frame.M * p_wrist_toolcom_);
    // compensate for the weight of the tool
    wrench_nog = wrench - gravity_transformation * base_tool_weight_com_;

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

  void FtSensorController::set_sensor_offset(std::vector<double>& wrench)
  {
    // set ft_sensor_offset_
    ft_sensor_offset_ = KDL::Wrench(KDL::Vector(wrench.at(0), wrench.at(1), wrench.at(2)), \
				    KDL::Vector(wrench.at(3), wrench.at(4), wrench.at(5)));
  }

  void FtSensorController::set_p_wrist_toolcom(std::vector<double>& p)
  {
    // set p_wrist_toolcom_
    p_wrist_toolcom_ = KDL::Vector(p.at(0), p.at(1), p.at(2));			
  }

  void FtSensorController::set_base_tool_weight_com(double mass)
  {
    // gravity_ defined in KinematicChainControllerBase
    base_tool_weight_com_ = KDL::Wrench(mass * gravity_, KDL::Vector::Zero());
  }
    
} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::FtSensorController , controller_interface::ControllerBase)
