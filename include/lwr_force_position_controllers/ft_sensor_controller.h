#ifndef LWR_CONTROLLERS_FT_SENSOR_CONTROLLER_H
#define LWR_CONTROLLERS_FT_SENSOR_CONTROLLER_H

#include <lwr_controllers/KinematicChainControllerBase.h>
#include <boost/scoped_ptr.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <lwr_force_position_controllers/FtSensorToolEstimation.h>
#include <lwr_force_position_controllers/FtSensorSetBias.h>
#include <yaml-cpp/yaml.h>

//KDL include
#include <kdl/chainfksolverpos_recursive.hpp>

namespace lwr_controllers
{
  class FtSensorController: public controller_interface::KinematicChainControllerBase<hardware_interface::JointStateInterface>
  {

  public:
    FtSensorController();
    ~FtSensorController();
    bool init(hardware_interface::JointStateInterface* robot, ros::NodeHandle& nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    
  private:
    void ft_sensor_topic_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void publish_data(KDL::Wrench wrench, ros::Publisher& pub);
    bool estimate_tool(lwr_force_position_controllers::FtSensorToolEstimation::Request& req,\
		       lwr_force_position_controllers::FtSensorToolEstimation::Response& res);
    bool get_estimated_tool(lwr_force_position_controllers::FtSensorToolEstimation::Request& req,\
			    lwr_force_position_controllers::FtSensorToolEstimation::Response& res);
    bool set_sensor_bias(lwr_force_position_controllers::FtSensorSetBias::Request& req, \
			 lwr_force_position_controllers::FtSensorSetBias::Response& res);
    void write_vector_to_yaml(std::string field, KDL::Wrench wrench);
    void write_vector_to_yaml(std::string field, KDL::Vector vector);
    void set_wrench(std::vector<double>& value, KDL::Wrench& wrench);
    void set_vector(std::vector<double>& p, KDL::Vector& vector);

    ros::Time last_publish_time_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    ros::Publisher  pub_ft_sensor_, pub_ft_sensor_nog_;
    ros::Subscriber sub_ft_sensor_;
    double publish_rate_;
    KDL::Wrench ft_sensor_bias_, ft_wrench_raw_, base_tool_weight_com_;
    KDL::Vector p_wrist_toolcom_;
    
    // set_sensor_initial_conditions service
    ros::ServiceServer estimate_tool_service_;
    ros::ServiceServer get_estimated_tool_service_;
    ros::ServiceServer set_sensor_bias_service_;
  };

} // namespace

#endif
