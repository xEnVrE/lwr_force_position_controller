#ifndef LWR_FORCE_POSITION_CONTROLLERS_HYBRID_IMPEDANCE_CONTROLLER_H
#define LWR_FORCE_POSITION_CONTROLLERS_HYBRID_IMPEDANCE_CONTROLLER_H

#include "cartesian_inverse_dynamics_controller.h"

// service
#include <lwr_force_position_controllers/SetHybridImpedanceCommand.h>

// msg include
#include <geometry_msgs/WrenchStamped.h>

namespace lwr_controllers
{
  class HybridImpedanceController: public CartesianInverseDynamicsController
  {
  public:
    
    HybridImpedanceController();
    ~HybridImpedanceController();
    
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

  private:
    bool set_cmd(lwr_force_position_controllers::SetHybridImpedanceCommand::Request &req, \
		 lwr_force_position_controllers::SetHybridImpedanceCommand::Response &res);
    void get_parameters(ros::NodeHandle &n);
    void set_circular_traj(const ros::Duration& period);
    void publish_data(ros::Publisher& pub, KDL::Wrench wrench);
    void publish_data(ros::Publisher& pub, Eigen::VectorXd& vector);

    // SetHybridImpiedanceCommand service
    ros::ServiceServer cmd_service_;
    
    // hybrid impedance controller (pose)
    Eigen::VectorXd x_des_, xdot_des_, xdotdot_des_;
    Eigen::MatrixXd Kp_, Kd_;    

    // hybrid impedance controller (force)
    double fz_des_;
    double km_f_, kd_f_;

    // circle trajectory
    bool circle_trj_;
    double circle_trj_frequency_;
    double circle_trj_radius_;
    double circle_trj_center_x_;
    double circle_trj_center_y_;
    double time_;

    // publisher to monitor data
    ros::Time last_publish_time_;
    double publish_rate_;
    ros::Publisher pub_force_, pub_force_des_, pub_state_, pub_dstate_;
    ros::Publisher pub_x_des_, pub_xdot_des_, pub_xdotdot_des_;
  };

} // namespace

#endif
