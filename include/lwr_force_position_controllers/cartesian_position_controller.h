#ifndef LWR_FORCE_POSITION_CONTROLLERS_CARTESIAN_POSITION_CONTROLLER_H
#define LWR_FORCE_POSITION_CONTROLLERS_CARTESIAN_POSITION_CONTROLLER_H

#include <lwr_controllers/KinematicChainControllerBase.h>
#include <lwr_force_position_controllers/CartesianPositionCommand.h>
#include <lwr_force_position_controllers/CartesianPositionCommandMsg.h>
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
    void evaluate_q_des(KDL::Vector&, KDL::Rotation&);
    bool set_cmd(lwr_force_position_controllers::CartesianPositionCommand::Request&,\
		 lwr_force_position_controllers::CartesianPositionCommand::Response&);
    bool get_cmd(lwr_force_position_controllers::CartesianPositionCommand::Request&,\
		 lwr_force_position_controllers::CartesianPositionCommand::Response&);

    // joint position controller
    double kp_, kd_;
   
    // inertia matrix and coriolis
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;

    // inverse kinematics q_des = ik(x_des)
    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    KDL::JntArray q_des_;

    // forward kinematics
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;

    // jacobian solver
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;

    // force and torques
    ros::Subscriber sub_force_;
    KDL::Wrench wrench_wrist_;

    // CartesianPositionCommand services
    ros::ServiceServer set_cmd_service_;
    ros::ServiceServer get_cmd_service_;

    // use simulation flag
    bool use_simulation_;

    // p_wrist_ee
    KDL::Vector p_wrist_ee_;

    // extended kdl chain
    KDL::Chain extended_chain_;

  };

} // namespace

#endif
