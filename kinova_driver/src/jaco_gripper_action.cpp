/**
 * @file jaco_trajectory_action.cpp
 *
 * @author Bc. Matěj Balga
 */

#include <kinova/KinovaTypes.h>
#include <cmath>
#include "kinova_driver/jaco_gripper_action.h"
#include "kinova_driver/jaco_types.h"

#define PI 3.14159265359
#define TOLERANCE 1 // Goal tolerance in degrees
#define KP 4 // P-regulator constant

namespace kinova
{

JacoGripperActionServer::JacoGripperActionServer(JacoComm &arm_comm, const ros::NodeHandle &nh)
  : arm_comm_(arm_comm),
    node_handle_(nh),
    action_server_(node_handle_, "gripper_action",
                   boost::bind(&JacoGripperActionServer::actionCallback, this, _1),
                   false)
{

  node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
  node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
  node_handle_.param<double>("rate_hz", rate_hz_, 10.0);
  node_handle_.param<double>("tolerance", tolerance_, 2.0);
  /*The angle in radians at which the gripper is supposed to be fully closed, so no object was grasped
  (fully open is 0, fully closed sould be 0.697 (40 Degrees) but sometimes the fingers fully close to even 0.79
  futher tests needed.)
  */
  node_handle_.param<double>("fully_closed_threshold", fully_closed_threshold_, 0.729);

  ROS_INFO_STREAM_NAMED("gripper", "Parameters: ");
  ROS_INFO_STREAM_NAMED("gripper", "stall_interval_seconds " << stall_interval_seconds_);
  ROS_INFO_STREAM_NAMED("gripper", "stall_threshold " << stall_threshold_);
  ROS_INFO_STREAM_NAMED("gripper", "rate_hz " << rate_hz_);
  ROS_INFO_STREAM_NAMED("gripper", "tolerance " << tolerance_);
  ROS_INFO_STREAM_NAMED("gripper", "fully_closed_threshold " << fully_closed_threshold_);
  
  // Approximative conversion ratio 
  // from finger position (0..6000) 
  // to joint angle in radians (0..0.697).
  const static double FULLY_CLOSED = 6600;
  const static double FULLY_CLOSED_URDF = M_PI/180*40; //0.697;
  encoder_to_radian_ratio_ = FULLY_CLOSED_URDF / FULLY_CLOSED;
  radian_to_encoder_ratio_ = FULLY_CLOSED / FULLY_CLOSED_URDF;

  arm_comm_.initFingers();
  ros::Duration(1).sleep();

  action_server_.start();
  ROS_INFO_STREAM("JACO Gripper action server has started.");
}

JacoGripperActionServer::~JacoGripperActionServer()
{
}

void JacoGripperActionServer::actionCallback(const control_msgs::GripperCommandGoalConstPtr &goal)
{
  std::cout << "actionCallback: " << *goal << std::endl;

  FingerAngles current_finger_positions;
  control_msgs::GripperCommandResult result;
  control_msgs::GripperCommandFeedback feedback;

  ros::Time current_time = ros::Time::now();

  try
  {
    arm_comm_.getFingerPositions(current_finger_positions);
    result.position = current_finger_positions.Finger1 * encoder_to_radian_ratio_;

    if (arm_comm_.isStopped())
    {
      ROS_INFO_STREAM_NAMED("gripper", "Could not complete finger action because the arm is stopped");
      //result.fingers = current_finger_positions.constructFingersMsg();
      action_server_.setAborted(result); //result);
      return;
    }

    last_nonstall_time_ = current_time;
    last_nonstall_finger_positions_ = current_finger_positions;

    kinova_msgs::FingerPosition finger_position;
    finger_position.finger1 = goal->command.position * radian_to_encoder_ratio_;
    finger_position.finger2 = goal->command.position * radian_to_encoder_ratio_;
    finger_position.finger3 = goal->command.position * radian_to_encoder_ratio_;
    FingerAngles target(finger_position);

    // Send command
    arm_comm_.setFingerPositions(target);

    // Loop until the action completed, is preempted, or fails in some way.
    // timeout is left to the caller since the timeout may greatly depend on
    // the context of the movement.
    while (true)
    {
      ros::spinOnce();

      arm_comm_.getFingerPositions(current_finger_positions);
      result.position = current_finger_positions.Finger1 * encoder_to_radian_ratio_;
      current_time = ros::Time::now();

      if (action_server_.isPreemptRequested() || !ros::ok())
      {
        ROS_WARN_STREAM_NAMED("gripper","Preempt requested");
        //result.fingers = current_finger_positions.constructFingersMsg();
        arm_comm_.stopAPI();
        arm_comm_.startAPI();
        action_server_.setPreempted(result);
        return;
      }

      if (arm_comm_.isStopped())
      {
        ROS_WARN_STREAM_NAMED("gripper","Arm is stopped");
        //result.fingers = current_finger_positions.constructFingersMsg();
        action_server_.setAborted(result);
        return;
      }

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

      //if (target.isCloseToOther(current_finger_positions, tolerance_))
      //{
      ROS_INFO_STREAM_NAMED("gripper","Wait 2 seconds for gripper to open/close");
      ros::Duration(2).sleep();

      ROS_DEBUG_STREAM_NAMED("gripper","Succeeded - positions are within tolerance");

      // Check if the action has succeeeded
      //result.fingers = current_finger_positions.constructFingersMsg();
      arm_comm_.getFingerPositions(current_finger_positions);
      result.position = current_finger_positions.Finger1 * encoder_to_radian_ratio_;

      if(result.position >= fully_closed_threshold_){

        ROS_ERROR_STREAM_NAMED("gripper","Gripper fully closed (fully closed threshold is set to " 
          << fully_closed_threshold_ << " current finger position is " 
          << result.position << " so no object was grasped. Aborting.");
        action_server_.setAborted(result);
        return;

      }

      result.reached_goal = true;
      action_server_.setSucceeded(result);
      ROS_INFO_STREAM_NAMED("gripper","Gripper command successfully executed.");
      return;

        /*
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
      }*/

      ros::Rate(rate_hz_).sleep();
    }
  }
  catch(const std::exception& e)
  {
    //result.fingers = current_finger_positions.constructFingersMsg();
    ROS_ERROR_STREAM_NAMED("gripper", "Error: " << e.what());
    action_server_.setAborted(result);
  }

  ROS_WARN_STREAM_NAMED("gripper","Should we have gotten here?");
}

} // namespace
