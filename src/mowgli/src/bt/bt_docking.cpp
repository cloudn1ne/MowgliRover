/*
 * Mowgli DOCKING Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */
#include "bt_docking.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#define BT_DEBUG 1

#define DOCKING_POINTS_PER_M   10.0

/// @brief Drive forward into docking station
BT::NodeStatus Docking::onStart()
{
      float docking_distance;
      std::string planner;

      getInput("docking_distance", docking_distance);
      getInput("planner", planner);

#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ Docking: STARTING ] docking_distance = "<< docking_distance << "m, planner = '" << planner << "'");
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
      nav_msgs::Path dockingPath;      
  
      uint16_t docking_point_count = docking_distance * DOCKING_POINTS_PER_M;
      for (int i = 0; i < docking_point_count; i++) {                  
            dockingPoseStamped.pose.position.x += cos(yaw) * (i / DOCKING_POINTS_PER_M);
            dockingPoseStamped.pose.position.y += sin(yaw) * (i / DOCKING_POINTS_PER_M);
#ifdef BT_DEBUG
            ROS_INFO_STREAM("[ Docking: STARTING ] docking path pose (" << i << ") x = "<< dockingPoseStamped.pose.position.x << " y= " << dockingPoseStamped.pose.position.y << " yaw = " << yaw*180/M_PI);
#endif            
            dockingPath.poses.push_back(dockingPoseStamped);
      }
 #ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ Docking: STARTING ] docking path consists of "<< dockingPath.poses.size() << " poses");
#endif

      mbf_msgs::ExePathGoal exePathGoal;
      exePathGoal.path = dockingPath;      
      exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.1;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = planner;
      
      _mbfClient->sendGoal(exePathGoal);
      
      return BT::NodeStatus::RUNNING;
}

/// @brief Monitor the current MBF Goal Execution
BT::NodeStatus Docking::onRunning()
{
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);

      current_status = _mbfClient->getState();

#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ Docking: STARTING ] v_charge = "<< *_v_charge << "V");
      printNavState(current_status.state_);
#endif
      switch (current_status.state_)
      {
            case actionlib::SimpleClientGoalState::SUCCEEDED: 
                                                              if (isDocked())                    
                                                              {
#ifdef BT_DEBUG                                                                  
                                                                  ROS_INFO_STREAM("Got a voltage of: " << *_v_charge << "V - docking is now completed");
                                                                  ROS_INFO_STREAM("[ DockingApproach: SUCCESS ]");
#endif                                                                                                                                    
                                                                  return BT::NodeStatus::SUCCESS; 
                                                              }
                                                              else
                                                              {
#ifdef BT_DEBUG                                                                                                                                   
                                                                  ROS_INFO_STREAM("[ DockingApproach: FAILURE (no voltage detected) ]");
#endif                                                                                                                                                                              
                                                                  return BT::NodeStatus::FAILURE;
                                                              }
            case actionlib::SimpleClientGoalState::ACTIVE:    
            case actionlib::SimpleClientGoalState::PENDING: 
                                                              if (isDocked())     
                                                              {
#ifdef BT_DEBUG                                                                  
                                                                 ROS_INFO_STREAM("Got a voltage of: " << *_v_charge << "V - aborting docking path");
                                                                 ROS_INFO_STREAM("[ DockingApproach: RUNNING (sensed voltage before path was completed) ]");
#endif                                                                 
                                                                 _mbfClient->cancelGoal();
                                                                 stopMoving(); // ensure the path planner does not interfere anymore or we might lose isDocked() again                                                                                                                                                                                                   
                                                              }
                                                              return BT::NodeStatus::RUNNING;            
            //----------------------------------------------------------------------------            
            case actionlib::SimpleClientGoalState::RECALLED: 
            case actionlib::SimpleClientGoalState::REJECTED: 
            case actionlib::SimpleClientGoalState::PREEMPTED: 
            case actionlib::SimpleClientGoalState::ABORTED: 
#ifdef BT_DEBUG
                                                              ROS_INFO_STREAM("[ DockingApproach: ABORTED ]");
#endif                                                                          
                                                              if (isDocked())
                                                              {
#ifdef BT_DEBUG                                                                  
                                                                 ROS_INFO_STREAM("Got a voltage of: " << *_v_charge << "V - docking was successfull after aborting docking path");                                                                 
#endif                                                                   
                                                                 return BT::NodeStatus::SUCCESS;
                                                              }
            case actionlib::SimpleClientGoalState::LOST:      return BT::NodeStatus::FAILURE;
                        
            default: ROS_ERROR_STREAM("[ Docking: onRunning ] MBF returned unknown state "<< current_status.state_);
      }
      
      // if we get here, something is wrong
      return BT::NodeStatus::FAILURE;
}

void Docking::onHalted() 
{
      // nothing to do here...
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ Docking: interrupted ]");    
#endif      
}

void Docking::printNavState(int state)
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
geometry_msgs::PoseStamped Docking::getDockingPose() {       
    geometry_msgs::PoseStamped _dockingPoseStamped;
    mower_map::GetDockingPointSrv srv;
    _svcClient.call(srv);
        
    _dockingPoseStamped.pose = srv.response.docking_pose;
    _dockingPoseStamped.header.frame_id = "map";
    _dockingPoseStamped.header.stamp = ros::Time::now();

    return(_dockingPoseStamped);
}

/// @brief return docking state, for now we use charge voltage detection as criteria
/// @return true if docked
bool Docking::isDocked()
{
      return (*_v_charge > 5.0);
}
