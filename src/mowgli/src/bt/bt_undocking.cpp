#include "bt_undocking.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// msg 
#include "mower_msgs/MowerControlSrv.h"

#define BT_DEBUG 1

#define UNDOCK_POINTS_PER_M   10.0

BT::NodeStatus Undocking::onStart()
{
      float undock_distance;
      getInput("undock_distance", undock_distance);
#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ Undocking: STARTING ] undock_distance = "<< undock_distance << "m");
#endif
      
      // get current yaw from /odom ()
      tf2::Quaternion quat;
      tf2::fromMsg(_odom.pose.pose.orientation, quat);
      tf2::Matrix3x3 matrix(quat);
      double roll, pitch, yaw;
      matrix.getRPY(roll, pitch, yaw);

      // create a path out of n individual poses       
      nav_msgs::Path undockingPath;
      geometry_msgs::PoseStamped undockingPose;
      undockingPose.pose = _odom.pose.pose;
      undockingPose.header = _odom.header;

      uint8_t undock_point_count = undock_distance * UNDOCK_POINTS_PER_M;
      for (int i = 0; i < undock_point_count; i++) {                  
            undockingPose.pose.position.x -= cos(yaw) * (i / UNDOCK_POINTS_PER_M);
            undockingPose.pose.position.y -= sin(yaw) * (i / UNDOCK_POINTS_PER_M);
            undockingPath.poses.push_back(undockingPose);
      }
#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ Undocking: STARTING ] undocking path consists of "<< undockingPath.poses.size() << " poses");
#endif
      mbf_msgs::ExePathGoal exePathGoal;
      exePathGoal.path = undockingPath;      
      exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.1;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "DockingFTCPlanner";
      
      _mbfClient->sendGoal(exePathGoal);
      
      return BT::NodeStatus::RUNNING;
}

// Monitor the current MBF Goal Execution
BT::NodeStatus Undocking::onRunning()
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
            case actionlib::SimpleClientGoalState::LOST:      return BT::NodeStatus::RUNNING;
                        
            default: ROS_ERROR_STREAM("[ Undocking: onRunning ] MBF returned unknown state "<< current_status.state_);
      }
      
      // if we get here, something is wrong
      return BT::NodeStatus::FAILURE;
}

void Undocking::onHalted() 
{
      // nothing to do here...
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ Undocking: interrupted ]");    
#endif      
}

void Undocking::printNavState(int state)
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
