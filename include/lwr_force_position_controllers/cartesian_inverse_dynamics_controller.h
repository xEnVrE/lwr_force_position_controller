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

    void get_gains_im(double& kp_z, double& kp_gamma, double& kd_pos, double& kd_att);
    void set_gains_im(double kp_z, double kp_gamma, double kd_pos, double kd_att);
    void set_p_wrist_ee(double x, double y, double z);
    void set_ft_sensor_topic_name(std::string topic);
    void set_p_base_ws(double x, double y, double z);
    void set_ws_base_angles(double alpha, double beta, double gamma);
    void set_command(Eigen::VectorXd& commanded_acceleration);
    void load_calib_data(double& total_mass, KDL::Vector& p_sensor_tool_com);
    void publish_torques();

  private:
    void force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void update_fri_inertia_matrix(Eigen::MatrixXd& fri_B);
    void extend_chain(ros::NodeHandle &n);

    // syntax:
    //
    // for jacobians x_J_y := Jacobian w.r.t reference point y expressed in basis x
    // for analytical jacobians x_JA_y := analytical Jacobian w.r.t reference point y expressed in basis x
    // for rotation matrices R_x_y := Rotation from basis y to basis x
    // for wrenches x_wrench_y := Wrench w.r.t reference point y expressed in basis x 
    // for vectors in general x_vector := vector expressed in basis x
    // p_x_y := arm from x to y
    // ee := reference point of interest (typically the tool tip)
    // wrist := tip of the 7th link of the Kuka LWR
    // T := euler kinematical matrix
    
    // possibly required by inheriting classes
    KDL::Rotation R_ws_base_;
    KDL::Rotation R_ws_ee_;
    KDL::Wrench base_wrench_wrist_;
    Eigen::VectorXd ws_x_, ws_xdot_;
    
    // setup by inheriting class
    KDL::Vector p_wrist_ee_;
    KDL::Vector p_base_ws_;

    // pointers to solvers
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> ee_jacobian_solver_, wrist_jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> ee_jacobian_dot_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> im_link4_jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> im_link4_fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> im_link5_jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> im_link5_fk_solver_;

    // chain required to move the reference point of jacobians
    KDL::Chain extended_chain_;
    // chain required to control internal motion
    KDL::Chain im_link4_chain_;
    KDL::Chain im_link5_chain_;

    // these matrices are sparse and initialized in init()
    Eigen::MatrixXd ws_TA_, ws_TA_dot_;
    Eigen::MatrixXd base_TA_im_link5_;

    // null space controller gains
    Eigen::Matrix<double, 6, 6> Kp_im_;
    Eigen::Matrix<double, 6, 6> Kd_im_;
    double gamma_im_link5_initial_;

    // commandss
    Eigen::VectorXd tau_fri_;
    Eigen::MatrixXd command_filter_;

    // ft sensor subscriber and related wrench
    std::string ft_sensor_topic_name_;

    ros::Subscriber sub_force_;
    KDL::Wrench wrench_wrist_;

    // use simulation flag
    bool use_simulation_;

    // publisher
    ros::Publisher pub_torques_;

  };

} // namespace

#endif
