/*
 * Mowgli MowPathApproachPoint Node v1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */
#include "bt_mowpathapproachpoint.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#define BT_DEBUG 1

/// @brief Approach Docking station (but does not actually dock)
BT::NodeStatus MowPathApproachPoint::onStart()
{
      geometry_msgs::PoseStamped approachPose;
      std::string planner;

      getInput("pose", approachPose);
      getInput("planner", planner);

#ifdef BT_DEBUG       
      // Calculate yaw (only for display purposes)
      tf2::Quaternion quat;
      tf2::fromMsg(approachPose.pose.orientation, quat);
      tf2::Matrix3x3 m(quat);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
 
      ROS_INFO_STREAM("[ MowPathApproachPoint: STARTING ] pose = x = "<< approachPose.pose.position.x << " y= " << approachPose.pose.position.y << " yaw = " << yaw*180/M_PI << " deg, planner = '" << planner << "'");
#endif
      
      // send docking approach pose to MBF
      mbf_msgs::MoveBaseGoal moveBaseGoal;
      moveBaseGoal.target_pose = approachPose;
      moveBaseGoal.controller = planner;      
      _mbfMoveBaseClient->sendGoal(moveBaseGoal);

      return BT::NodeStatus::RUNNING;    
}

/// @brief Monitor the current MBF Goal Execution
BT::NodeStatus MowPathApproachPoint::onRunning()
{
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);

      current_status = _mbfMoveBaseClient->getState();

#ifdef BT_DEBUG        
      printNavState(current_status.state_);
#endif
      switch (current_status.state_)
      {
            case actionlib::SimpleClientGoalState::SUCCEEDED: 
#ifdef BT_DEBUG
                                                            ROS_INFO_STREAM("[ MowPathApproachPoint: SUCCESS ]");
#endif            

                                                            return BT::NodeStatus::SUCCESS;
            //----------------------------------------------------------------------------
            case actionlib::SimpleClientGoalState::PENDING: 
            case actionlib::SimpleClientGoalState::ACTIVE:  
#ifdef BT_DEBUG
                                                            ROS_INFO_STREAM("[ MowPathApproachPoint: RUNNING ]");
#endif                          
                                                            return BT::NodeStatus::RUNNING;
            //----------------------------------------------------------------------------            
            case actionlib::SimpleClientGoalState::RECALLED: 
            case actionlib::SimpleClientGoalState::REJECTED: 
            case actionlib::SimpleClientGoalState::PREEMPTED: 
            case actionlib::SimpleClientGoalState::ABORTED: 
            case actionlib::SimpleClientGoalState::LOST:    return BT::NodeStatus::FAILURE;
                        
            default: ROS_ERROR_STREAM("[ MowPathApproachPoint: onRunning ] MBF returned unknown state "<< current_status.state_);
      }
      
      // if we get here, something is wrong
      return BT::NodeStatus::FAILURE;
}

void MowPathApproachPoint::onHalted() 
{
      // nothing to do here...
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ MowPathApproachPoint: interrupted ]");    
#endif      
}

void MowPathApproachPoint::printNavState(int state)
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
