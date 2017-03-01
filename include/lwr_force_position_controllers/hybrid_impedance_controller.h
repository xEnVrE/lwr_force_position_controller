#ifndef LWR_FORCE_POSITION_CONTROLLERS_HYBRID_IMPEDANCE_CONTROLLER_H
#define LWR_FORCE_POSITION_CONTROLLERS_HYBRID_IMPEDANCE_CONTROLLER_H

#include "cartesian_inverse_dynamics_controller.h"
#include <lwr_force_position_controllers/HybridImpedanceCommand.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread/mutex.hpp>

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
    bool set_cmd(lwr_force_position_controllers::HybridImpedanceCommand::Request &req, \
		 lwr_force_position_controllers::HybridImpedanceCommand::Response &res);
    bool get_cmd(lwr_force_position_controllers::HybridImpedanceCommand::Request &req, \
		 lwr_force_position_controllers::HybridImpedanceCommand::Response &res);
    void get_parameters(ros::NodeHandle &n);
    /* void eval_current_circular_traj(const ros::Duration& period); */
    void eval_current_point_to_point_traj(const ros::Duration& period,\
					  Eigen::VectorXd& x_des,\
					  Eigen::VectorXd& xdot_des,\
					  Eigen::VectorXd& xdotdot_des);
    void eval_point_to_point_traj_constants(Eigen::Vector3d& desired_position,\
					    Eigen::Vector3d& desired_attitude,\
					    double duration);
    void evaluate_force_reference_constants(double force_des, double duration);
    double eval_force_reference(const ros::Duration& period);
    void publish_data(ros::Publisher& pub, KDL::Wrench wrench);
    void publish_data(ros::Publisher& pub, Eigen::VectorXd& vector);

    // HybridImpiedanceCommand service
    ros::ServiceServer set_cmd_service_;
    ros::ServiceServer get_cmd_service_;
    
    // hybrid impedance controller (pose)
    Eigen::MatrixXd Kp_, Kd_;
    double p2p_traj_duration_;
    Eigen::MatrixXf p2p_trj_const_;

    // hybrid impedance controller (force)
    double fz_des_final_;
    double km_f_, kd_f_;
    double force_ref_duration_;
    Eigen::VectorXf force_ref_const_;
    double time_force_;

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
    ros::Publisher pub_error_;

    //
    boost::mutex p2p_traj_mutex_;
    boost::mutex force_traj_mutex_;
  };

} // namespace

#endif
