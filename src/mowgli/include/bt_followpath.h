#ifndef BT_FOLLOWPATH_H
#define BT_FOLLOWPATH_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"    
#include "actionlib/client/simple_action_client.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Transform.h>
#include "mower_map/GetDockingPointSrv.h"
#include "ftc_local_planner/PlannerGetProgress.h"

class FollowPath : public BT::StatefulActionNode
{
  public:    
    FollowPath(const std::string& name, const BT::NodeConfiguration& config,               
               actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfExePathClient,
               ros::ServiceClient svcPlannerGetProgressClient                  
               )
      : StatefulActionNode(name, config), 
      _mbfExePathClient(mbfExePathClient),
      _svcPlannerGetProgressClient(svcPlannerGetProgressClient)
    {        
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<nav_msgs::Path>("path"),
                BT::InputPort<std::string>("planner"), 
                BT::OutputPort<int>("current_index")
              };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;

    virtual void printNavState(int state);

    virtual int getCurrentMowPathIndex();

  private:            
    actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *_mbfExePathClient;  
    ros::ServiceClient _svcPlannerGetProgressClient;
    nav_msgs::Path _path;
        
};

#endif // BT_FOLLOWPATH_H
