#ifndef LWR_FORCE_POSITION_CONTROLLERS_HYBRID_IMPEDANCE_CONTROLLER_H
#define LWR_FORCE_POSITION_CONTROLLERS_HYBRID_IMPEDANCE_CONTROLLER_H

#include "cartesian_inverse_dynamics_controller.h"
#include <lwr_force_position_controllers/HybridImpedanceCommandTrajPos.h>
#include <lwr_force_position_controllers/HybridImpedanceCommandTrajForce.h>
#include <lwr_force_position_controllers/HybridImpedanceCommandGains.h>
#include <lwr_force_position_controllers/HybridImpedanceSwitchForcePos.h>
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
    void set_default_pos_traj();
    void set_default_force_traj();
    bool set_cmd_traj_pos(lwr_force_position_controllers::HybridImpedanceCommandTrajPos::Request &req, \
			  lwr_force_position_controllers::HybridImpedanceCommandTrajPos::Response &res);
    bool set_cmd_traj_force(lwr_force_position_controllers::HybridImpedanceCommandTrajForce::Request &req, \
			    lwr_force_position_controllers::HybridImpedanceCommandTrajForce::Response &res);
    bool set_cmd_gains(lwr_force_position_controllers::HybridImpedanceCommandGains::Request &req, \
		       lwr_force_position_controllers::HybridImpedanceCommandGains::Response &res);
    bool get_cmd_traj_pos(lwr_force_position_controllers::HybridImpedanceCommandTrajPos::Request &req, \
			  lwr_force_position_controllers::HybridImpedanceCommandTrajPos::Response &res);
    bool get_cmd_traj_force(lwr_force_position_controllers::HybridImpedanceCommandTrajForce::Request &req, \
			    lwr_force_position_controllers::HybridImpedanceCommandTrajForce::Response &res);
    bool get_cmd_gains(lwr_force_position_controllers::HybridImpedanceCommandGains::Request &req, \
		       lwr_force_position_controllers::HybridImpedanceCommandGains::Response &res);
    bool switch_force_position_z(lwr_force_position_controllers::HybridImpedanceSwitchForcePos::Request &req, \
				 lwr_force_position_controllers::HybridImpedanceSwitchForcePos::Response &res);
    void get_parameters(ros::NodeHandle &n);
    void set_p_sensor_cp(double x, double y, double z);
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
    ros::ServiceServer set_cmd_traj_pos_service_;
    ros::ServiceServer set_cmd_traj_force_service_;
    ros::ServiceServer set_cmd_gains_service_;
    ros::ServiceServer switch_force_position_z_service_;
    ros::ServiceServer get_cmd_traj_pos_service_;
    ros::ServiceServer get_cmd_traj_force_service_;
    ros::ServiceServer get_cmd_gains_service_;
    
    // hybrid impedance controller (pose)
    Eigen::MatrixXd Kp_, Kd_;
    double p2p_traj_duration_;
    Eigen::MatrixXf p2p_trj_const_;
    Eigen::Vector3d prev_pos_setpoint_;
    Eigen::Vector3d prev_att_setpoint_;
    double time_;

    // hybrid impedance controller (force)
    double prev_fz_setpoint_;
    double km_f_, kd_f_;
    double force_ref_duration_;
    Eigen::VectorXf force_ref_const_;
    double time_force_;
    KDL::Vector p_sensor_cp_;

    // publisher to monitor data
    ros::Time last_publish_time_;
    double publish_rate_;
    ros::Publisher pub_force_, pub_force_des_, pub_state_, pub_dstate_;
    ros::Publisher pub_x_des_, pub_xdot_des_, pub_xdotdot_des_;
    ros::Publisher pub_error_;

    // 
    boost::mutex p2p_traj_mutex_;
    boost::mutex force_traj_mutex_;

    // switch between z force control and z position control
    bool enable_force_;
  };

} // namespace

#endif
