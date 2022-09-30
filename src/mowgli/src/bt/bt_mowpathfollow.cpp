/*
 * Mowgli MowPathFollow Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */

#include "bt_mowpathfollow.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#define BT_DEBUG 1

#define DOCKING_POINTS_PER_M   10.0

/// @brief Approach Docking station (but does not actually dock)
BT::NodeStatus MowPathFollow::onStart()
{
      nav_msgs::Path mowPath;            
      std::string planner;

      getInput("mowpath", mowPath);
      getInput("planner", planner);

#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ MowPathFollow: STARTING ] mowpath poses = "<< mowPath.poses.size() << "m, planner = '" << planner << "'");
#endif

      mbf_msgs::ExePathGoal exePathGoal;
      exePathGoal.path = mowPath;
      exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.1;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = planner;
      
      _mbfExePathClient->sendGoal(exePathGoal);

      return BT::NodeStatus::RUNNING;
}

/// @brief Monitor the current MBF Goal Execution
BT::NodeStatus MowPathFollow::onRunning()
{

      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);

      current_status = _mbfExePathClient->getState();

#ifdef BT_DEBUG        
      printNavState(current_status.state_);
#endif
      switch (current_status.state_)
      {
            case actionlib::SimpleClientGoalState::SUCCEEDED: 
#ifdef BT_DEBUG
                                                            ROS_INFO_STREAM("[ MowPathFollow: SUCCESS ]");
#endif            

                                                            return BT::NodeStatus::SUCCESS;
            //----------------------------------------------------------------------------
            case actionlib::SimpleClientGoalState::PENDING: 
            case actionlib::SimpleClientGoalState::ACTIVE:  
#ifdef BT_DEBUG
                                                            ROS_INFO_STREAM("[ MowPathFollow: RUNNING ]");
#endif                          
                                                            return BT::NodeStatus::RUNNING;
            //----------------------------------------------------------------------------            
            case actionlib::SimpleClientGoalState::RECALLED: 
            case actionlib::SimpleClientGoalState::REJECTED: 
            case actionlib::SimpleClientGoalState::PREEMPTED: 
            case actionlib::SimpleClientGoalState::ABORTED: 
            case actionlib::SimpleClientGoalState::LOST:      return BT::NodeStatus::FAILURE;
                        
            default: ROS_ERROR_STREAM("[ MowPathFollow: onRunning ] MBF returned unknown state "<< current_status.state_);
      }
      
      // if we get here, something is wrong
      return BT::NodeStatus::FAILURE;
}

void MowPathFollow::onHalted() 
{
      // nothing to do here...
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ MowPathFollow: interrupted ]");    
#endif      
}

void MowPathFollow::printNavState(int state)
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
