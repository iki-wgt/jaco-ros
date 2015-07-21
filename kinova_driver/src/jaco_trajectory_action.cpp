/**
 * @file jaco_trajectory_action.cpp
 *
 * @author Bc. Matěj Balga
 */

#include <kinova/KinovaTypes.h>
#include <cmath>
#include "kinova_driver/jaco_trajectory_action.h"
#include "kinova_driver/jaco_types.h"

#define PI 3.14159265359
#define TOLERANCE 1 // Goal tolerance in degrees

namespace kinova 
{

JacoTrajectoryActionServer::JacoTrajectoryActionServer(JacoComm &arm_comm, const ros::NodeHandle &nh)
  : arm_comm_(arm_comm),
    node_handle_(nh),
    action_server_(node_handle_,
                   "follow_joint_trajectory",
                   boost::bind(&JacoTrajectoryActionServer::actionCallback, this, _1),
                   false)
{

  /*Sets the p value for the velocity controlled end position correction, can be seen as speed */
  node_handle_.param<double>("controller_p_value", controller_p_value_, 2.0);

  double arm_stale_threshold = 0;
  node_handle_.param<double>("arm_stale_threshold", arm_stale_threshold, 4.0);

  arm_stale_threshold_ = ros::Duration(arm_stale_threshold);

  ROS_INFO_STREAM_NAMED("arm", "controller_p_value " << controller_p_value_);
  ROS_INFO_STREAM_NAMED("arm", "arm_stale_threshold " << arm_stale_threshold_);

  action_server_.start();
  ROS_INFO_STREAM("JACO FollowJointTrajectory action server has started.");
}

JacoTrajectoryActionServer::~JacoTrajectoryActionServer()
{
}

void JacoTrajectoryActionServer::actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  //std::cout << "goal: " << *goal << std::endl;

  ROS_INFO_STREAM_NAMED("trajectory_action", "Executing arm trajectory...");

  try
  {
    if (arm_comm_.isStopped())
    {
      ROS_ERROR_STREAM("Could not complete joint angle action because the arm is 'stopped'.");
      action_server_.setAborted();
      return;
    }

    // Clear all previous trajectories and points
    arm_comm_.initTrajectory();

    trajectory_msgs::JointTrajectoryPoint goal_waypoint = goal->trajectory.points.back();
      
    // Error check
    if (goal_waypoint.positions.size() < 6)
    {
      ROS_ERROR_STREAM_NAMED("trajectory_action","Insufficient joints passed in, expecting 6 but recieved " 
                             << goal_waypoint.positions.size());
      action_server_.setAborted();
    }

    JacoAngles current_joint_angles, goal_joint_angles;
    goal_joint_angles.Actuator1 = goal_waypoint.positions[0] * (180 / PI);
    goal_joint_angles.Actuator2 = goal_waypoint.positions[1] * (180 / PI);
    goal_joint_angles.Actuator3 = goal_waypoint.positions[2] * (180 / PI);
    goal_joint_angles.Actuator4 = goal_waypoint.positions[3] * (180 / PI);
    goal_joint_angles.Actuator5 = goal_waypoint.positions[4] * (180 / PI);
    goal_joint_angles.Actuator6 = goal_waypoint.positions[5] * (180 / PI);
    convertDHAnglesToPhysical(goal_joint_angles);
    normalizeAngles(goal_joint_angles);

    ros::Time start_time = ros::Time::now();

    for (unsigned int i = 0; i < goal->trajectory.points.size()-1; i++)
    {

      // Get the waypoint to be reached
      trajectory_msgs::JointTrajectoryPoint waypoint =
        goal->trajectory.points[i];

      // Initialize a trajectory point which we will be constantly
      // feeding to the robot until the waypoint time's up or until
      // we are in the final configuration.
      TrajectoryPoint point;
      point.InitStruct();
      memset(&point, 0, sizeof (point));

      // The trajectory consists of angular velocity waypoints
      point.Position.Type = ANGULAR_VELOCITY;

      // Set up the trajectory point with waypoint values converted
      // from [rad/s] to [deg/s]
      point.Position.Actuators.Actuator1 = -waypoint.velocities[0] * (180 / PI);
      point.Position.Actuators.Actuator2 = +waypoint.velocities[1] * (180 / PI);
      point.Position.Actuators.Actuator3 = -waypoint.velocities[2] * (180 / PI);
      point.Position.Actuators.Actuator4 = -waypoint.velocities[3] * (180 / PI);
      point.Position.Actuators.Actuator5 = -waypoint.velocities[4] * (180 / PI);
      point.Position.Actuators.Actuator6 = -waypoint.velocities[5] * (180 / PI);

      ros::Rate r(100);   // The loop below will run at 100Hz

      while ((waypoint.time_from_start >= ros::Time::now() - start_time))
      {
        ros::spinOnce();

        // If preempted, stop the motion, enable the arm again and
        // send a notification of preemption
        if (action_server_.isPreemptRequested() || !ros::ok())
        {
          arm_comm_.stopAPI();
          arm_comm_.startAPI();
          action_server_.setPreempted();
          ROS_INFO_STREAM_NAMED("trajectory_action", "Preempted by preempt request");
          return;
        } else if (arm_comm_.isStopped())
        {
          action_server_.setAborted();
          ROS_INFO_STREAM_NAMED("trajectory_action", "Aborted because arm is stopped.");
          return;
        }

        // Get the current real angles and normalize them for comparing
        // with the target and adjusting the execution
        arm_comm_.getJointAngles(current_joint_angles);
        normalizeAngles(current_joint_angles);

        // Adjust actuator velocities accordingly
        arm_comm_.addTrajectoryPoint(point);

        // Ensures executing at 100Hz
        r.sleep();
      }
    }

    bool stop = false;

    ros::Rate r(100); // The loop below will run at 100Hz (every 4ms)

    ROS_INFO_STREAM_NAMED("trajectory_action", "Starting end position optimization routine ");

    start_time = ros::Time::now();

    // This loop finalizes the movement by checking angular distance
    // of joints from the desired configuration. Instead of moving
    // to the last waypoint, we move and check each joint separately
    // to ensure we are at the right position.
    while (!stop)
    {

      if (action_server_.isPreemptRequested() || !ros::ok())
      {
        arm_comm_.stopAPI();
        arm_comm_.startAPI();
        action_server_.setPreempted();
        ROS_INFO_STREAM_NAMED("trajectory_action", "Preempted by preempt request");
        return;
      }

      if (arm_stale_threshold_ < ros::Time::now() - start_time)
      {
        ROS_INFO_STREAM_NAMED("trajectory_action", "Set succeeded but staled after waiting " << arm_stale_threshold_ << " seconds.");
        action_server_.setSucceeded();
        return;

      }

      // Get the current real angles and normalize them for comparing
      // with the target and adjusting the execution
      arm_comm_.getJointAngles(current_joint_angles);
      normalizeAngles(current_joint_angles);
      AngularInfo error = computeError(goal_joint_angles, current_joint_angles);

      // Initialize a trajectory point which we will be constantly
      // feeding to the robot untilwe are in the final configuration.
      TrajectoryPoint point;
      point.InitStruct();
      memset(&point, 0, sizeof (point));

      point.Position.Actuators.Actuator1 = 0.0;
      point.Position.Actuators.Actuator2 = 0.0;
      point.Position.Actuators.Actuator3 = 0.0;
      point.Position.Actuators.Actuator4 = 0.0;
      point.Position.Actuators.Actuator5 = 0.0;
      point.Position.Actuators.Actuator6 = 0.0;

      // If no adjustment is made, then stop
      stop = true;

      // The trajectory consists of angular velocity waypoints
      point.Position.Type = ANGULAR_VELOCITY;

      // If not in the position yet, adjust the first motor
      if (std::abs(error.Actuator1) > TOLERANCE)
      {
        point.Position.Actuators.Actuator1 = controller_p_value_ * error.Actuator1;
        stop = false;
      }

      // If not in the position yet, adjust the second motor
      if (std::abs(error.Actuator2) > TOLERANCE)
      {
        point.Position.Actuators.Actuator2 = controller_p_value_ * error.Actuator2;
        stop = false;
      }

      // If not in the position yet, adjust the third motor
      if (std::abs(error.Actuator3) > TOLERANCE)
      {
        point.Position.Actuators.Actuator3 = controller_p_value_ * error.Actuator3;
        stop = false;
      }

      // If not in the position yet, adjust the fourth motor
      if (std::abs(error.Actuator4) > TOLERANCE)
      {
        point.Position.Actuators.Actuator4 = controller_p_value_ * error.Actuator4;
        stop = false;
      }

      // If not in the position yet, adjust the fifth motor
      if (std::abs(error.Actuator5) > TOLERANCE)
      {
        point.Position.Actuators.Actuator5 = controller_p_value_ * error.Actuator5;
        stop = false;
      }

      // If not in the position yet, adjust the sixth motor
      if (std::abs(error.Actuator6) > TOLERANCE)
      {
        point.Position.Actuators.Actuator6 = controller_p_value_ * error.Actuator6;
        stop = false;
      }

      // Adjust actuator velocities accordingly
      arm_comm_.addTrajectoryPoint(point);

      // Ensure execution at 100Hz
      r.sleep();
    }

    // If we got here, it seems that we succeeded
    ROS_INFO_STREAM_NAMED("trajectory_action", "Successfully executed Trajectory. SetSucceeded!");
    action_server_.setSucceeded();

  } catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    ROS_INFO_STREAM_NAMED("trajectory_action", "Something terrible has happened");
    action_server_.setAborted();
  }
}

/**
 * Converts angles from DH convention to JACO's physical angles.
 *
 * @param angles Angles to be converted
 */
void JacoTrajectoryActionServer::convertDHAnglesToPhysical(AngularInfo &angles)
{
  angles.Actuator1 = 180 - angles.Actuator1;
  angles.Actuator2 = 270 + angles.Actuator2;
  angles.Actuator3 =  90 - angles.Actuator3;
  angles.Actuator4 = 180 - angles.Actuator4;
  angles.Actuator5 = 180 - angles.Actuator5;
  angles.Actuator6 = 270 - angles.Actuator6;
}

/**
 * Converts angles from JACO's physical angles to DH convention.
 *
 * @param angles Angles to be converted
 */
void JacoTrajectoryActionServer::convertPhysicalAnglesToDH(AngularInfo &angles)
{
  angles.Actuator1 = 180 - angles.Actuator1;
  angles.Actuator2 = angles.Actuator2 - 270;
  angles.Actuator3 =  90 - angles.Actuator3;
  angles.Actuator4 = 180 - angles.Actuator4;
  angles.Actuator5 = 180 - angles.Actuator5;
  angles.Actuator6 = 270 - angles.Actuator6;
}

/**
 * Returns the difference between two joint configurations.
 *
 * @param goal The goal configuration.
 * @param current Current configuration.
 * @return Difference joint configuration.
 */
AngularInfo JacoTrajectoryActionServer::computeError(AngularInfo &reference, AngularInfo &current)
{
  AngularInfo error;

  error.Actuator1 = getShortestAngleDistance(reference.Actuator1, current.Actuator1);
  error.Actuator2 = getShortestAngleDistance(reference.Actuator2, current.Actuator2);
  error.Actuator3 = getShortestAngleDistance(reference.Actuator3, current.Actuator3);
  error.Actuator4 = getShortestAngleDistance(reference.Actuator4, current.Actuator4);
  error.Actuator5 = getShortestAngleDistance(reference.Actuator5, current.Actuator5);
  error.Actuator6 = getShortestAngleDistance(reference.Actuator6, current.Actuator6);
  return error;
}

float JacoTrajectoryActionServer::getShortestAngleDistance(float &reference, float &current)
{
  float error = reference - current;

  // Go the shortest way: if the error is bigger than PI, let's take
  // the other way around (typical e.g. for reference=-179 and feedback=179)
  // error in that case would be 358 but in fact those positions are
  // just 2 degrees from each other
  if (std::abs(error) > 180)
  {
    error = (error > 0 ? (reference - current) - 360 : 360 - (current - reference));
  }

  return error;
}

/**
 * Normalizes the angles to lie within -180 to 180 degrees.
 *
 * @param angles
 */
void JacoTrajectoryActionServer::normalizeAngles(AngularInfo &angles)
{
  angles.Actuator1 = normalize(angles.Actuator1, -180.0, 180.0);
  angles.Actuator2 = normalize(angles.Actuator2, -180.0, 180.0);
  angles.Actuator3 = normalize(angles.Actuator3, -180.0, 180.0);
  angles.Actuator4 = normalize(angles.Actuator4, -180.0, 180.0);
  angles.Actuator5 = normalize(angles.Actuator5, -180.0, 180.0);
  angles.Actuator6 = normalize(angles.Actuator6, -180.0, 180.0);
}

// DEBUG FUNCTION
void JacoTrajectoryActionServer::printAngles(const char* desc, AngularInfo &angles)
{
  char wp_info[1024];
  sprintf(wp_info, "%s:\t[%f, %f, %f, %f, %f, %f]",
          desc,
          angles.Actuator1,
          angles.Actuator2,
          angles.Actuator3,
          angles.Actuator4,
          angles.Actuator5,
          angles.Actuator6);
  ROS_INFO_STREAM(wp_info);
}

/**
 * Normalizes any number to an arbitrary range by assuming the range wraps
 * around when going below min or above max.
 *
 * @param value The number to be normalized
 * @param start Lower threshold
 * @param end   Upper threshold
 * @return Returns the normalized number.
 */
double JacoTrajectoryActionServer::normalize(const double value, const double start, const double end)
{
  const double width = end - start; //
  const double offsetValue = value - start; // value relative to 0

  return ( offsetValue - (floor(offsetValue / width) * width)) +start;
  // + start to reset back to start of original range
}

/**
 * Returns true if the distance of the values lies within _tolerance_.
 *
 * @param first The first value
 * @param second The second value
 * @param tolerance Maximum distance of the values
 * @return TRUE if the distance is less than tolerance
 */
bool JacoTrajectoryActionServer::areValuesClose(float first, float second, float tolerance)
{
  return ((first <= second + tolerance) && (first >= second - tolerance));
}

} // namespace
