#ifndef LWR_FORCE_POSITION_CONTROLLERS_CARTESIAN_INVERSE_DYNAMICS_CONTROLLER_H
#define LWR_FORCE_POSITION_CONTROLLERS_CARTESIAN_INVERSE_DYNAMICS_CONTROLLER_H

#include <lwr_controllers/KinematicChainControllerBase.h>

#include <boost/scoped_ptr.hpp>

#include <geometry_msgs/WrenchStamped.h>

//KDL include
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdljacdot/chainjnttojacdotsolver.hpp>

namespace lwr_controllers
{
  class CartesianInverseDynamicsController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
  {
    friend class HybridImpedanceController;

  public:
    CartesianInverseDynamicsController();
    ~CartesianInverseDynamicsController();
    
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void set_p_wrist_ee(double x, double y, double z);
    void set_p_base_ws(double x, double y, double z);
    void set_p_wrist_ee_com(double x, double y, double z);
    void set_ws_base_angles(double alpha, double beta, double gamma);
    void set_initial_ft_sensor_wrench(KDL::Wrench wrench);
    void set_command(Eigen::VectorXd& commanded_acceleration);
    void set_tool_weight(double);

  private:
    void force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    // syntax:
    //
    // for jacobians x_J_y := Jacobian w.r.t reference point y expressed in basis x
    // for analytical jacobians x_JA_y := analytical Jacobian w.r.t reference point y expressed in basis x
    // for rotation matrices R_x_y := Rotation from basis y to basis x
    // ee := reference point of interest (typically the tool tip)
    // T := euler kinematical matrix
    
    KDL::JntArray C_, q0_;
    KDL::JntSpaceInertiaMatrix B_;
    KDL::Jacobian base_J_wrist_;
    KDL::Rotation R_ws_base_;
    KDL::Rotation R_ws_ee_;
    KDL::Wrench wrench_wrist_, base_wrench_wrist_, base_weight_com_;
    KDL::Vector p_wrist_ee_;
    KDL::Vector p_base_ws_;
    KDL::Vector p_wrist_ee_com_;
    KDL::Frame ee_fk_frame_;
    KDL::Chain extended_chain_;
    KDL::Wrench ee_calibration_wrench_;

    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> ee_jacobian_solver_, wrist_jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> ee_jacobian_dot_solver_;

    Eigen::VectorXd ws_x_, ws_xdot_;
    Eigen::Matrix<double, 6,1> base_F_wrist_;
    Eigen::MatrixXd ws_TA_, ws_TA_dot_;
    Eigen::MatrixXd ws_JA_ee_dot_;
    Eigen::MatrixXd BA_;
    Eigen::MatrixXd ws_JA_ee_;

    ros::Subscriber sub_force_;
  };

} // namespace

#endif
