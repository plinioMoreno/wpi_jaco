#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/foreach.hpp>
//#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <ecl/geometry.hpp>
/*#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/EStop.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/HomeArmAction.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/QuaternionToEuler.h>*/
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>


/*!
 * \class MicoArmVrepTrajectoryController
 * \brief Provides for trajectory execution and gripper control of the MICO arm on V-REP simulator.
 *
 * MicoArmVrepTrajectoryController creates a ROS node that provides trajectory execution and gripper 
 * control through the vrep_ros_bridge package.
 */

class MicoArmVrepTrajectoryController
{
public:
  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;
  typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction>        GripperServer;

  /**
   * \brief Constructor
   * @param nh ROS node handle
   * @param pnh ROS private node handle
   */
  MicoArmVrepTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh);

  /**
   * \brief Destructor
   */
  virtual ~MicoArmVrepTrajectoryController();

  /**
   * \brief Callback for the arm_controller, executes a joint angle trajectory
   * @param goal action goal
   */
  void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  /**
   *\brief Stripped-down angular trajectory point sending to the arm
   *
   * This is designed for trajectory followers, which need a quick response
   * @param point angular trajectory point to send to the arm
   * @param erase if true, clear the trajectory point stack before sending point
   */
  //void executeAngularTrajectoryPoint(TrajectoryPoint point, bool erase);

  ros::Timer joint_state_timer_; //!< timer for joint state publisher

  // Actionlib
  TrajectoryServer*  trajectory_server_; //!< point-to-point trajectory follower


  ros::Publisher pub_joint_commands_;
  sensor_msgs::JointState jointcommands;
  std::vector<std::string> joint_names;
  int           num_joints_;

};

