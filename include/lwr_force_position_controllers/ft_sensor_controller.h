#ifndef LWR_FORCE_POSITION_CONTROLLERS_FT_SENSOR_CONTROLLER_H
#define LWR_FORCE_POSITION_CONTROLLERS_FT_SENSOR_CONTROLLER_H

#include <lwr_controllers/KinematicChainControllerBase.h>
#include <boost/scoped_ptr.hpp>
#include <geometry_msgs/WrenchStamped.h>

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
    void set_sensor_offset(std::vector<double>& wrench);
    void set_p_wrist_toolcom(std::vector<double>& p);
    void set_base_tool_weight_com(double mass);
    void publish_data(KDL::Wrench wrench, ros::Publisher& pub);

    ros::Time last_publish_time_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    ros::Publisher  pub_ft_sensor_, pub_ft_sensor_nog_;
    ros::Subscriber sub_ft_sensor_;
    double publish_rate_;
    KDL::Wrench ft_sensor_offset_, ft_wrench_raw_, base_tool_weight_com_;
    KDL::Vector p_wrist_toolcom_;

  };

} // namespace

#endif
