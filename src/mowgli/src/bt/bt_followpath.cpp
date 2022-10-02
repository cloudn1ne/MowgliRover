/*
 * Mowgli FollowPath Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 * Arguments:
 * 
 *    <FollowPath path="{mowpath}" planner="FTCPlanner" current_index="{mowpath_index}"/> 
 * 
 * Description:
 * 
 *    Follow the given path using planner. current_index will be updated as the bot travels along the path
 * 
 *    For this node to return SUCCESS, the path needs to be completed.
 * 
 */

#include "bt_followpath.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#define BT_DEBUG 1

#define DOCKING_POINTS_PER_M   10.0

/// @brief Approach Docking station (but does not actually dock)
BT::NodeStatus FollowPath::onStart()
{             
      std::string planner;

      getInput("path", _path);      // store in private var, as we need it for onRunning() updates
      getInput("planner", planner);

#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ FollowPath: STARTING ] mowpath poses = "<< _path.poses.size() << ", planner = '" << planner << "'");
#endif

      // check if we have the minimum amount of poses nessecary in our path
      if (_path.poses.size() < 2)
      {
#ifdef BT_DEBUG        
            ROS_INFO_STREAM("[ FollowPath: STARTING ] error not enough poses for a path");
#endif
            return BT::NodeStatus::FAILURE;
      }

      mbf_msgs::ExePathGoal exePathGoal;
      exePathGoal.path = _path;
      exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.1;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = planner;
      
      _mbfExePathClient->sendGoal(exePathGoal);

      return BT::NodeStatus::RUNNING;
}

/// @brief Monitor the current MBF Goal Execution
BT::NodeStatus FollowPath::onRunning()
{
      int currentIndex = getCurrentMowPathIndex()+1;    // mow progress 
      setOutput("current_index", currentIndex);
      
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      current_status = _mbfExePathClient->getState();

#ifdef BT_DEBUG        
      printNavState(current_status.state_);
#endif
      switch (current_status.state_)
      {
            case actionlib::SimpleClientGoalState::SUCCEEDED: 
#ifdef BT_DEBUG
                                                            ROS_INFO_STREAM("[ FollowPath: SUCCESS " << currentIndex <<  "/" << _path.poses.size() << " ]");
#endif            
                                                            // the global planner sometimes errors out with
                                                            // Robot is far away from global plan. It probably has crashed.
                                                            // which sets MBF to SUCCEEDED state despite not having completed the path
                                                            // so we capture this here and return FAILURE instead of SUCCESS
                                                            if (currentIndex >= _path.poses.size())
                                                            {
                                                                  return BT::NodeStatus::SUCCESS;     // path fully done
                                                            }
                                                            else
                                                            {
                                                                  return BT::NodeStatus::FAILURE;     // premature "error"
                                                            }
            //----------------------------------------------------------------------------
            case actionlib::SimpleClientGoalState::PENDING: 
            case actionlib::SimpleClientGoalState::ACTIVE:  
#ifdef BT_DEBUG
                                                            ROS_INFO_STREAM("[ FollowPath: RUNNING " << currentIndex <<  "/" << _path.poses.size() << " ]");
#endif                          
                                                            return BT::NodeStatus::RUNNING;
            //----------------------------------------------------------------------------            
            case actionlib::SimpleClientGoalState::RECALLED: 
            case actionlib::SimpleClientGoalState::REJECTED: 
            case actionlib::SimpleClientGoalState::PREEMPTED: 
            case actionlib::SimpleClientGoalState::ABORTED: 
            case actionlib::SimpleClientGoalState::LOST:      
#ifdef BT_DEBUG
                                                            ROS_INFO_STREAM("[ FollowPath: FAILURE " << currentIndex <<  "/" << _path.poses.size() << " ]");
#endif            
                                                            return BT::NodeStatus::FAILURE;
                        
            default: ROS_ERROR_STREAM("[ FollowPath: onRunning ] MBF returned unknown state "<< current_status.state_ << " pose "<< currentIndex <<  "/" << _path.poses.size() << " ]");
      }
      
      // if we get here, something is wrong
      return BT::NodeStatus::FAILURE;
}

void FollowPath::onHalted() 
{
      // stop MBF
      _mbfExePathClient->cancelAllGoals();
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ FollowPath: interrupted ]");    
#endif      
}

void FollowPath::printNavState(int state)
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

int FollowPath::getCurrentMowPathIndex()
{
    ftc_local_planner::PlannerGetProgress progress;
    int currentIndex = -1;
    if(_svcPlannerGetProgressClient.call(progress)) {
        currentIndex = progress.response.index;
    } else {
        ROS_ERROR("mowgli_bt: FollowPath/getCurrentMowPathIndex() - Error getting progress from FTC planner");
    }
    return(currentIndex);
}