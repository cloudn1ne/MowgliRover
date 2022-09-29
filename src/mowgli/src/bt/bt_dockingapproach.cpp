/*
 * Mowgli DOCKINGAPPROACH Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */

#include "bt_dockingapproach.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#define BT_DEBUG 1

#define DOCKING_POINTS_PER_M   10.0

/// @brief Approach Docking station (but does not actually dock)
BT::NodeStatus DockingApproach::onStart()
{
 float docking_approach_distance;
      std::string planner;

      getInput("docking_approach_distance", docking_approach_distance);
      getInput("planner", planner);

#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ DockingApproach: STARTING ] docking_approach_distance = "<< docking_approach_distance << "m, planner = '" << planner << "'");
#endif

        // get docking pose
      geometry_msgs::PoseStamped dockingPoseStamped = getDockingPose();

      // Calculate a docking approaching point behind the actual docking point
      tf2::Quaternion quat;
      tf2::fromMsg(dockingPoseStamped.pose.orientation, quat);
      tf2::Matrix3x3 m(quat);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);


      // create a path out of n individual poses       
      nav_msgs::Path dockingApproachPath;      

      // we start at the DockingApproachPoint, and we use <dockingapproach_point_count> poses to sneak up to our dockingPose 
      uint8_t dockingapproach_point_count = docking_approach_distance * DOCKING_POINTS_PER_M;
      for (int i = 0; i < dockingapproach_point_count; i++) {          
            geometry_msgs::PoseStamped dockingApproachPose = dockingPoseStamped;        
            dockingApproachPose.pose.position.x -= cos(yaw) * ((dockingapproach_point_count - i) / DOCKING_POINTS_PER_M);
            dockingApproachPose.pose.position.y -= sin(yaw) * ((dockingapproach_point_count - i) / DOCKING_POINTS_PER_M);                   
#ifdef BT_DEBUG
            ROS_INFO_STREAM("[ DockingApproach: STARTING ] path pose (" << i << ") x = "<< dockingApproachPose.pose.position.x << " y= " << dockingApproachPose.pose.position.y << " yaw = " << yaw*180/M_PI);
#endif                 
            dockingApproachPath.poses.push_back(dockingApproachPose);             
      }
#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ DockingApproach: STARTING ] docking approach path consists of "<< dockingApproachPath.poses.size() << " poses");
#endif

      mbf_msgs::ExePathGoal exePathGoal;
      exePathGoal.path = dockingApproachPath;
      exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.1;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = planner;
      
      _mbfExePathClient->sendGoal(exePathGoal);

      return BT::NodeStatus::RUNNING;
}

/// @brief Monitor the current MBF Goal Execution
BT::NodeStatus DockingApproach::onRunning()
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
                                                            ROS_INFO_STREAM("[ DockingApproach: SUCCESS ]");
#endif            

                                                            return BT::NodeStatus::SUCCESS;
            //----------------------------------------------------------------------------
            case actionlib::SimpleClientGoalState::PENDING: 
            case actionlib::SimpleClientGoalState::ACTIVE:  
#ifdef BT_DEBUG
                                                            ROS_INFO_STREAM("[ DockingApproach: RUNNING ]");
#endif                          
                                                            return BT::NodeStatus::RUNNING;
            //----------------------------------------------------------------------------            
            case actionlib::SimpleClientGoalState::RECALLED: 
            case actionlib::SimpleClientGoalState::REJECTED: 
            case actionlib::SimpleClientGoalState::PREEMPTED: 
            case actionlib::SimpleClientGoalState::ABORTED: 
            case actionlib::SimpleClientGoalState::LOST:      return BT::NodeStatus::FAILURE;
                        
            default: ROS_ERROR_STREAM("[ DockingApproach: onRunning ] MBF returned unknown state "<< current_status.state_);
      }
      
      // if we get here, something is wrong
      return BT::NodeStatus::FAILURE;
}

void DockingApproach::onHalted() 
{
      // nothing to do here...
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ DockingApproach: interrupted ]");    
#endif      
}

void DockingApproach::printNavState(int state)
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



/// @brief Get Docking Pose (2nd X press) from Map Server
geometry_msgs::PoseStamped DockingApproach::getDockingPose() {       
    geometry_msgs::PoseStamped dockingPoseStamped;
    mower_map::GetDockingPointSrv srv;
    _svcClient.call(srv);
        
    dockingPoseStamped.pose = srv.response.docking_pose;
    dockingPoseStamped.header.frame_id = "map";
    dockingPoseStamped.header.stamp = ros::Time::now();

    return(dockingPoseStamped);
}
