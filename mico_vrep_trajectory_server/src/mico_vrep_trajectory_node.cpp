#include <mico_vrep_trajectory_node.h>

using namespace std;
/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 */
/*static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}*/

MicoArmVrepTrajectoryController::MicoArmVrepTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  // Create servers
  trajectory_server_              = new TrajectoryServer( nh, "/mico_arm/joint_velocity_controller/trajectory", boost::bind(&MicoArmVrepTrajectoryController::execute_trajectory, this, _1), true);

  pub_joint_commands_ =
    nh.advertise<sensor_msgs::JointState>(
    "/vrep/Mico/jointCommand", 1, true);


/*'Mico_joint1', 'Mico_joint2', 'Mico_joint3', 'Mico_joint4', 'Mico_joint5', 'Mico_joint6', 'MicoHand_fingers12_motor1', 'MicoHand_fingers12_motor2', 'MicoHand_joint1_finger3', 'MicoHand_Prismatic_joint5', 'MicoHand_joint2_finger3', 'MicoHand_Prismatic_joint3', 'MicoHand_joint1_finger1', 'MicoHand_Prismatic_joint1', 'MicoHand_joint2_finger1', 'MicoHand_Prismatic_joint2'*/
  jointcommands.name.push_back("Mico_joint1");
  joint_names.push_back("Mico_joint1");
  jointcommands.name.push_back("Mico_joint2");
  joint_names.push_back("Mico_joint2");
  jointcommands.name.push_back("Mico_joint3");
  joint_names.push_back("Mico_joint3");
  jointcommands.name.push_back("Mico_joint4");
  joint_names.push_back("Mico_joint4");
  jointcommands.name.push_back("Mico_joint5");
  joint_names.push_back("Mico_joint5");
  jointcommands.name.push_back("Mico_joint6");
  joint_names.push_back("Mico_joint6");
  jointcommands.name.push_back("MicoHand_fingers12_motor1");
  joint_names.push_back("MicoHand_fingers12_motor11");
  jointcommands.name.push_back("MicoHand_fingers12_motor2");
  joint_names.push_back("MicoHand_fingers12_motor2");
  jointcommands.name.push_back("MicoHand_joint1_finger3");
  joint_names.push_back("MicoHand_joint1_finger31");
  jointcommands.name.push_back("MicoHand_Prismatic_joint5");
  joint_names.push_back("MicoHand_Prismatic_joint5");
  jointcommands.name.push_back("MicoHand_joint2_finger3");
  joint_names.push_back("MicoHand_joint2_finger3");
  jointcommands.name.push_back("MicoHand_Prismatic_joint3");
  joint_names.push_back("MicoHand_Prismatic_joint3");
  jointcommands.name.push_back("MicoHand_joint1_finger1");
  joint_names.push_back("MicoHand_joint1_finger1");
  jointcommands.name.push_back("MicoHand_Prismatic_joint1");
  joint_names.push_back("MicoHand_Prismatic_joint1");
  jointcommands.name.push_back("MicoHand_joint2_finger1");
  joint_names.push_back("MicoHand_joint2_finger1");
  jointcommands.name.push_back("MicoHand_Prismatic_joint2");
  joint_names.push_back("MicoHand_Prismatic_joint");

  this->num_joints_ = jointcommands.name.size();
  jointcommands.position.resize(this->num_joints_);
  jointcommands.velocity.resize(this->num_joints_);
  jointcommands.effort.resize(this->num_joints_);

}

MicoArmVrepTrajectoryController::~MicoArmVrepTrajectoryController()
{
}
void MicoArmVrepTrajectoryController::execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{

//    boost::recursive_mutex::scoped_lock lock(api_mutex);

//  update_joint_states();
  /*double current_joint_pos[NUM_JACO_JOINTS];

  AngularPosition position_data;
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetAngularPosition(position_data);
  }
  current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
  current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
  current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
  current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
  current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
  current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;

  //initialize trajectory point
  TrajectoryPoint trajPoint;
  trajPoint.InitStruct();
  trajPoint.Position.Type = ANGULAR_POSITION;
  trajPoint.Position.HandMode = HAND_NOMOVEMENT;*/
  BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points)
  {
    ROS_INFO("Trajectory Point");
    //double joint_cmd[NUM_JACO_JOINTS];
    for (unsigned int i = 0; i < this->num_joints_; i++)
    {
	jointcommands.position[i]     = 0;
	jointcommands.velocity[i]     = 0;
	jointcommands.effort[i]       = 0;
    }
    for (int trajectory_index = 0; trajectory_index < goal->trajectory.joint_names.size(); ++trajectory_index)
    {
      string joint_name = goal->trajectory.joint_names[trajectory_index];
      int joint_index = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));
      /*if (joint_index >= 0 && joint_index < NUM_JACO_JOINTS)
      {
        //ROS_INFO("Before: %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
        if (joint_index != 1 && joint_index != 2)
          joint_cmd[joint_index] = nearest_equivalent(simplify_angle(point.positions[trajectory_index]),
                                                      current_joint_pos[joint_index]);
        else
          joint_cmd[joint_index] = point.positions[trajectory_index];
        //ROS_INFO("After:  %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, joint_cmd[joint_index]);
      }*/
    jointcommands.position[joint_index]=point.positions[trajectory_index];
    }

/*
    for (int i = 0; i < NUM_JACO_JOINTS; i++)
      current_joint_pos[i] = joint_cmd[i];

    AngularInfo angles;
    angles.Actuator1 = joint_cmd[0] * RAD_TO_DEG;
    angles.Actuator2 = joint_cmd[1] * RAD_TO_DEG;
    angles.Actuator3 = joint_cmd[2] * RAD_TO_DEG;
    angles.Actuator4 = joint_cmd[3] * RAD_TO_DEG;
    angles.Actuator5 = joint_cmd[4] * RAD_TO_DEG;
    angles.Actuator6 = joint_cmd[5] * RAD_TO_DEG;

    trajPoint.Position.Actuators = angles;

    executeAngularTrajectoryPoint(trajPoint, false);
*/
  pub_joint_commands_.publish(jointcommands);
  }

  /*ros::Rate rate(10);
  int trajectory_size;
  while (trajectory_size > 0)
  {
    //cancel check
    if (eStopEnabled)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      trajectory_server_->setSucceeded(result);
      return;
    }

    //check for preempt requests from clients
    if (trajectory_server_->isPreemptRequested() || !ros::ok())
    {
      //stop gripper control
      trajPoint.Position.Type = ANGULAR_VELOCITY;
      trajPoint.Position.Actuators.Actuator1 = 0.0;
      trajPoint.Position.Actuators.Actuator2 = 0.0;
      trajPoint.Position.Actuators.Actuator3 = 0.0;
      trajPoint.Position.Actuators.Actuator4 = 0.0;
      trajPoint.Position.Actuators.Actuator5 = 0.0;
      trajPoint.Position.Actuators.Actuator6 = 0.0;
      executeAngularTrajectoryPoint(trajPoint, true);

      //preempt action server
      trajectory_server_->setPreempted();
      ROS_INFO("Joint trajectory server preempted by client");

      return;
    }

    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);

      TrajectoryFIFO Trajectory_Info;
      memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
      GetGlobalTrajectoryInfo(Trajectory_Info);
      trajectory_size = Trajectory_Info.TrajectoryCount;
    }
    //ROS_INFO("%f, %f, %f, %f, %f, %f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
    rate.sleep();
  }
*/

  ROS_INFO("Trajectory Control Complete.");
  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  trajectory_server_->setSucceeded(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mico_arm_vrep_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  MicoArmVrepTrajectoryController robot(nh, pnh);
  
  ros::spin();
}
