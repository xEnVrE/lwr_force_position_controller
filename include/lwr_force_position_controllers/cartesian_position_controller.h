#ifndef LWR_FORCE_POSITION_CONTROLLERS_CARTESIAN_POSITION_CONTROLLER_H
#define LWR_FORCE_POSITION_CONTROLLERS_CARTESIAN_POSITION_CONTROLLER_H

#include <lwr_controllers/KinematicChainControllerBase.h>
#include <lwr_force_position_controllers/CartesianPositionCommandGains.h>
#include <lwr_force_position_controllers/CartesianPositionCommandGainsMsg.h>
#include <lwr_force_position_controllers/CartesianPositionCommandTraj.h>
#include <lwr_force_position_controllers/CartesianPositionCommandTrajMsg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/scoped_ptr.hpp>

/*
  Cartesian Position Controller law is:

  tau_cmd_ = B(q) * y + C * dq + J' * F
  y =  - Kp * q - Kd * dq + r
  r = Kp * q_d
  q_d = inverse_kinematics(x_d)
*/

namespace lwr_controllers
{

  class CartesianPositionController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
  {
  public:

    CartesianPositionController();
    ~CartesianPositionController();
     		
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

  private:
    void ft_sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void evaluate_traj_constants(KDL::Vector&, KDL::Rotation&);
    void evaluate_traj_des(const ros::Duration& period);
    bool set_cmd_traj(lwr_force_position_controllers::CartesianPositionCommandTraj::Request&, \
		      lwr_force_position_controllers::CartesianPositionCommandTraj::Response&);
    bool set_cmd_gains(lwr_force_position_controllers::CartesianPositionCommandGains::Request&, \
		      lwr_force_position_controllers::CartesianPositionCommandGains::Response&);
    bool get_cmd_traj(lwr_force_position_controllers::CartesianPositionCommandTraj::Request&,\
		      lwr_force_position_controllers::CartesianPositionCommandTraj::Response&);
    bool get_cmd_gains(lwr_force_position_controllers::CartesianPositionCommandGains::Request&,\
		      lwr_force_position_controllers::CartesianPositionCommandGains::Response&);
    void publish_data(ros::Publisher& pub, KDL::JntArray& array);
    void print_joint_array(KDL::JntArray& array);
    void update_fri_inertia_matrix(Eigen::MatrixXd& fri_B);
    
    // joint position controller
    double kp_, kp_a5_, kp_a6_, kd_;
   
    // inertia matrix and coriolis
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;

    // inverse kinematics q_des = ik(x_des)
    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

    // forward kinematics
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;

    // jacobian solver
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;

    // desired trajectory 
    // s(t) = a5 * t^5 + a4 * t^4 + a3 * t^3 + a0
    KDL::JntArrayAcc traj_des_;
    KDL::JntArray traj_a0_, traj_a3_, traj_a4_, traj_a5_;
    KDL::JntArray prev_q_setpoint_;
    double time_;
    double p2p_traj_duration_;

    // force and torques
    ros::Subscriber sub_force_;
    KDL::Wrench wrench_wrist_;

    // CartesianPositionCommand services
    ros::ServiceServer set_cmd_gains_service_;
    ros::ServiceServer set_cmd_traj_service_;
    ros::ServiceServer get_cmd_gains_service_;
    ros::ServiceServer get_cmd_traj_service_;

    // use simulation flag
    bool use_simulation_;

    // p_wrist_ee
    KDL::Vector p_wrist_ee_;

    // extended kdl chain
    KDL::Chain extended_chain_;

    // publisher to monitor data
    ros::Time last_publish_time_;
    double publish_rate_;
    ros::Publisher pub_error_, pub_q_des_;

  };

} // namespace

#endif
