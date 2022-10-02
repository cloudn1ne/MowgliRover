/*
 * Mowgli DRIVEFORWARD Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 * 
 * Arguments:
 * 
 *    <DriveForward distance="<distance_in_meter>" planner="<planner_name>"/>
 * 
 * Description:
 * 
 *    Calculate a path and drive distance forward from the current pose.
 *
 *    For this node to return SUCCESS, the end pose of the path needs to be reached..
 * 
 */
 
#include "bt_driveforward.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1

#define POSES_PER_M   10.0

/// @brief Drive backwards from the current posistion for <distance> meters using <planner>
/// @return NodeStatus
BT::NodeStatus DriveForward::onStart()
{
      float distance;
      std::string planner;

      getInput("distance", distance);
      getInput("planner", planner);

#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ DriveForward: STARTING ] distance = "<< distance << "m");      
#endif
      
      // get current yaw from /odom ()
      tf2::Quaternion quat;
      tf2::fromMsg(_odom->pose.pose.orientation, quat);
      tf2::Matrix3x3 matrix(quat);
      double roll, pitch, yaw;
      matrix.getRPY(roll, pitch, yaw);

#ifdef BT_DEBUG
      ROS_INFO_STREAM("[ DriveForward: STARTING ] current pose x = "<< _odom->pose.pose.position.x << " y= " << _odom->pose.pose.position.y << " yaw = " << yaw*180/M_PI);
#endif

      // create a path out of n individual poses       
      nav_msgs::Path DriveForwardPath;

      geometry_msgs::PoseStamped DriveForwardPose;
      DriveForwardPose.header = _odom->header;           
      
      uint16_t point_count = distance * POSES_PER_M;
      for (int i = 0; i < point_count; i++) {    
            DriveForwardPose.pose = _odom->pose.pose; 
            DriveForwardPose.pose.position.x += cos(yaw) * (i / POSES_PER_M);
            DriveForwardPose.pose.position.y += sin(yaw) * (i / POSES_PER_M);
#ifdef BT_DEBUG
            ROS_INFO_STREAM("[ DriveForward: STARTING ] path pose (" << i << ") x = "<< DriveForwardPose.pose.position.x << " y= " << DriveForwardPose.pose.position.y << " yaw = " << yaw*180/M_PI);
#endif            

            DriveForwardPath.poses.push_back(DriveForwardPose);
      }
#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ DriveForward: STARTING ] DriveForward path consists of "<< DriveForwardPath.poses.size() << " poses");
#endif
      mbf_msgs::ExePathGoal exePathGoal;
      exePathGoal.path = DriveForwardPath;      
      exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.1;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = planner;
      
      _mbfExePathClient->sendGoal(exePathGoal);
      
      return BT::NodeStatus::RUNNING;
}

// Monitor the current MBF Goal Execution
BT::NodeStatus DriveForward::onRunning()
{
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);

      current_status = _mbfExePathClient->getState();

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
                        
            default: ROS_ERROR_STREAM("[ DriveForward: onRunning ] MBF returned unknown state "<< current_status.state_);
      }
      
      // if we get here, something is wrong
      return BT::NodeStatus::FAILURE;
}

void DriveForward::onHalted() 
{
        // stop MBF
      _mbfExePathClient->cancelAllGoals();
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ DriveForward: interrupted ]");    
#endif      
}

void DriveForward::printNavState(int state)
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