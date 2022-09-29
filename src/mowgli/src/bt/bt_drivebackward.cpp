/*
 * Mowgli DRIVEBACKWARD Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */

#include "bt_drivebackward.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1

#define POINTS_PER_M   10.0

/// @brief Drive backwards from the current posistion for <distance> meters using <planner>
/// @return NodeStatus
BT::NodeStatus DriveBackward::onStart()
{
      float distance;
      std::string planner;

      getInput("distance", distance);
      getInput("planner", planner);

#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ DriveBackward: STARTING ] distance = "<< distance << "m");      
#endif
      
      // get current yaw from /odom ()
      tf2::Quaternion quat;
      tf2::fromMsg(_odom->pose.pose.orientation, quat);
      tf2::Matrix3x3 matrix(quat);
      double roll, pitch, yaw;
      matrix.getRPY(roll, pitch, yaw);

#ifdef BT_DEBUG
      ROS_INFO_STREAM("[ DriveBackward: STARTING ] current pose x = "<< _odom->pose.pose.position.x << " y= " << _odom->pose.pose.position.y << " yaw = " << yaw*180/M_PI);
#endif

      // create a path out of n individual poses       
      nav_msgs::Path DriveBackwardPath;

      geometry_msgs::PoseStamped DriveBackwardPose;
      DriveBackwardPose.header = _odom->header;

      uint16_t point_count = distance * POINTS_PER_M;
      for (int i = 0; i < point_count; i++) {    
            DriveBackwardPose.pose = _odom->pose.pose;              
            DriveBackwardPose.pose.position.x -= cos(yaw) * (i / POINTS_PER_M);
            DriveBackwardPose.pose.position.y -= sin(yaw) * (i / POINTS_PER_M);
#ifdef BT_DEBUG
            ROS_INFO_STREAM("[ DriveBackward: STARTING ] path pose (" << i << ") x = "<< DriveBackwardPose.pose.position.x << " y= " << DriveBackwardPose.pose.position.y << " yaw = " << yaw*180/M_PI);
#endif
            DriveBackwardPath.poses.push_back(DriveBackwardPose);
      }
#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ DriveBackward: STARTING ] DriveBackward path consists of "<< DriveBackwardPath.poses.size() << " poses");
#endif
      mbf_msgs::ExePathGoal exePathGoal;
      exePathGoal.path = DriveBackwardPath;      
      exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.1;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = planner;
      
      _mbfClient->sendGoal(exePathGoal);
      
      return BT::NodeStatus::RUNNING;
}

// Monitor the current MBF Goal Execution
BT::NodeStatus DriveBackward::onRunning()
{
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);

      current_status = _mbfClient->getState();

#ifdef BT_DEBUG        
      printNavState(current_status.state_);
#endif
      switch (current_status.state_)
      {
            case actionlib::SimpleClientGoalState::SUCCEEDED: return BT::NodeStatus::SUCCESS;
            //----------------------------------------------------------------------------
            case actionlib::SimpleClientGoalState::PENDING: 
            case actionlib::SimpleClientGoalState::ACTIVE:    return BT::NodeStatus::RUNNING;
            //----------------------------------------------------------------------------            
            case actionlib::SimpleClientGoalState::RECALLED: 
            case actionlib::SimpleClientGoalState::REJECTED: 
            case actionlib::SimpleClientGoalState::PREEMPTED: 
            case actionlib::SimpleClientGoalState::ABORTED: 
            case actionlib::SimpleClientGoalState::LOST:      return BT::NodeStatus::FAILURE;
                        
            default: ROS_ERROR_STREAM("[ DriveBackward: onRunning ] MBF returned unknown state "<< current_status.state_);
      }
      
      // if we get here, something is wrong
      return BT::NodeStatus::FAILURE;
}

void DriveBackward::onHalted() 
{
      // nothing to do here...
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ DriveBackward: interrupted ]");    
#endif      
}

void DriveBackward::printNavState(int state)
{
    switch (state)
    {
        case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
        case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
        case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
        case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
        case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
        case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
        case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
        case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
        default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
    }
}