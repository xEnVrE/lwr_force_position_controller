#ifndef LWR_CONTROLLERS_FT_SENSOR_CALIB_CONTROLLER_H
#define LWR_CONTROLLERS_FT_SENSOR_CALIB_CONTROLLER_H

#include <lwr_controllers/KinematicChainControllerBase.h>
#include <boost/scoped_ptr.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl/chainiksolverpos_lma.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_srvs/Empty.h>
#include <ft_calib/ft_calib.h>

namespace lwr_controllers
{
  class FtSensorCalibController: public controller_interface::KinematicChainControllerBase<hardware_interface::JointStateInterface>
  {

  public:
    FtSensorCalibController();
    ~FtSensorCalibController();
    bool init(hardware_interface::JointStateInterface* robot, ros::NodeHandle& nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    
  private:
    void ft_raw_topic_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    bool move_next_calib_pose(std_srvs::Empty::Request&,\
			      std_srvs::Empty::Response&);
    bool move_home_pose(std_srvs::Empty::Request&,\
			std_srvs::Empty::Response&);
    void send_joint_trajectory_msg(std::vector<double> q_des);
    void get_calibration_poses(ros::NodeHandle);
    void get_calibration_q(ros::NodeHandle nh);

    bool save_calib_data(std_srvs::Empty::Request&,\
		      std_srvs::Empty::Response&);
    void recover_existing_data();
    void save_calib_meas(KDL::Vector gravity, KDL::Wrench ft_wrench_avg, int index);

    bool do_estimation_step(std_srvs::Empty::Request&,\
		      std_srvs::Empty::Response&);
    void add_measurement(KDL::Vector gravity_ft, KDL::Wrench ft_raw_avg);


    KDL::Wrench ft_wrench_raw_;

    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    boost::scoped_ptr<Calibration::FTCalib> ft_calib_;

    std::vector<KDL::Frame> ft_calib_poses_;
    std::vector<std::vector<double>> ft_calib_q_;
    std::vector<double> ft_calib_q_home_;

    ros::ServiceServer move_home_pose_service_;
    ros::ServiceServer move_next_calib_pose_service_;
    ros::ServiceServer do_estimation_step_service_;
    ros::ServiceServer save_calib_data_service_;
    ros::Subscriber sub_ft_sensor_;
    ros::Publisher pub_joint_traj_ctl_;
    
    Eigen::Vector3d p_sensor_tool_com_;
    Eigen::Vector3d ft_offset_force_;
    Eigen::Vector3d ft_offset_torque_;

    double tool_mass_;
    double calibration_loop_rate_;

    int pose_counter_;
    int number_of_poses_;
  };

} // namespace

#endif
