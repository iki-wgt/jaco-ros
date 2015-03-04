/**
 * @file jaco_trajectory_action.cpp
 *
 * @author Bc. Matěj Balga
 */

#include <kinova/KinovaTypes.h>
#include <cmath>
#include "jaco_driver/jaco_gripper_action.h"
#include "jaco_driver/jaco_types.h"

#define PI 3.14159265359
#define TOLERANCE 1 // Goal tolerance in degrees
#define KP 4 // P-regulator constant

namespace jaco
{

JacoGripperActionServer::JacoGripperActionServer(JacoComm &arm_comm, const ros::NodeHandle &nh)
  : arm_comm_(arm_comm),
    node_handle_(nh, "fingers"),
    action_server_(node_handle_, "gripper_action",
                   boost::bind(&JacoGripperActionServer::actionCallback, this, _1),
                   false)
{
  double tolerance;
  node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
  node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
  node_handle_.param<double>("rate_hz", rate_hz_, 10.0);
  node_handle_.param<double>("tolerance", tolerance, 2.0);
  tolerance_ = static_cast<float>(tolerance);

  action_server_.start();
  ROS_INFO_STREAM("JACO Gripper action server has started.");
}

JacoGripperActionServer::~JacoGripperActionServer()
{
}

void JacoGripperActionServer::actionCallback(const control_msgs::GripperCommandGoalConstPtr &goal)
{
  FingerAngles current_finger_positions;
  ros::Time current_time = ros::Time::now();

  try
  {
    arm_comm_.getFingerPositions(current_finger_positions);

    if (arm_comm_.isStopped())
    {
      ROS_INFO("Could not complete finger action because the arm is stopped");
      //result.fingers = current_finger_positions.constructFingersMsg();
      action_server_.setAborted(); //result);
      return;
    }

    last_nonstall_time_ = current_time;
    last_nonstall_finger_positions_ = current_finger_positions;

    // The range of input for all three fingers on the JACO is approximately 0 (fully open) to 60 (fully closed)
    const static double FULLY_CLOSED = 6400;
    const static double FULLY_CLOSED_URDF = 0.697;

    double position = goal->command.position * FULLY_CLOSED / FULLY_CLOSED_URDF;

    jaco_msgs::FingerPosition finger_position;
    finger_position.finger1 = position;
    finger_position.finger2 = position;
    finger_position.finger3 = position;
    FingerAngles target(finger_position);

    // Send command
    arm_comm_.setFingerPositions(target);

    // Loop until the action completed, is preempted, or fails in some way.
    // timeout is left to the caller since the timeout may greatly depend on
    // the context of the movement.
    while (true)
    {
      ros::spinOnce();

      if (action_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_DEBUG_STREAM_NAMED("gripper","Preempt requested");
        //result.fingers = current_finger_positions.constructFingersMsg();
        arm_comm_.stopAPI();
        arm_comm_.startAPI();
        action_server_.setPreempted(); //result);
        return;
      }
      else if (arm_comm_.isStopped())
      {
        ROS_DEBUG_STREAM_NAMED("gripper","Arm is stopped");
        //result.fingers = current_finger_positions.constructFingersMsg();
        action_server_.setAborted(); //result);
        return;
      }

      arm_comm_.getFingerPositions(current_finger_positions);
      current_time = ros::Time::now();
      //feedback.fingers = current_finger_positions.constructFingersMsg();
      //action_server_.publishFeedback(feedback);

      // Debug
      if (false)
      {
        std::cout << "Target finger: " << target.constructFingersMsg().finger1 << " current: " << current_finger_positions.constructFingersMsg().finger1 << std::endl;
        std::cout << "Target finger: " << target.constructFingersMsg().finger2 << " current: " << current_finger_positions.constructFingersMsg().finger2 << std::endl;
        std::cout << "Target finger: " << target.constructFingersMsg().finger3 << " current: " << current_finger_positions.constructFingersMsg().finger3 << std::endl;
        std::cout << "Tolerance: " << tolerance_ << std::endl;
      }

      if (target.isCloseToOther(current_finger_positions, tolerance_))
      {
        ROS_DEBUG_STREAM_NAMED("gripper","Succeeded - positions are within tolerance");

        // Check if the action has succeeeded
        //result.fingers = current_finger_positions.constructFingersMsg();
        action_server_.setSucceeded(); //result);
        return;
      }
      else if (!last_nonstall_finger_positions_.isCloseToOther(current_finger_positions, stall_threshold_))
      {
        //ROS_DEBUG_STREAM_NAMED("gripper","Not close enough yet but not stalled yet");
        // Check if we are outside of a potential stall condition
        last_nonstall_time_ = current_time;
        last_nonstall_finger_positions_ = current_finger_positions;
      }
      else if ((current_time - last_nonstall_time_).toSec() > stall_interval_seconds_)
      {
        ROS_DEBUG_STREAM_NAMED("gripper","Has been stalled over " << stall_interval_seconds_ << " seconds");

        // Check if the full stall condition has been meet
        //result.fingers = current_finger_positions.constructFingersMsg();
        arm_comm_.stopAPI();
        arm_comm_.startAPI();
        action_server_.setPreempted(); //result);
        return;
      }

      ros::Rate(rate_hz_).sleep();
    }
  }
  catch(const std::exception& e)
  {
    //result.fingers = current_finger_positions.constructFingersMsg();
    ROS_ERROR_STREAM(e.what());
    action_server_.setAborted(); //result);
  }

  ROS_WARN_STREAM_NAMED("gripper","Should we have gotten here?");
}

} // namespace
