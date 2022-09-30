#ifndef BT_MOWPATHFOLLOW_H
#define BT_MOWPATHFOLLOW_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"    
#include "actionlib/client/simple_action_client.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Transform.h>
#include "mower_map/GetDockingPointSrv.h"

class MowPathFollow : public BT::StatefulActionNode
{
  public:    
    MowPathFollow(const std::string& name, const BT::NodeConfiguration& config,               
               actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfExePathClient                             
               )
      : StatefulActionNode(name, config), 
      _mbfExePathClient(mbfExePathClient)
    {        
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<nav_msgs::Path>("mowpath"),
                BT::InputPort<std::string>("planner") 
              };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;

    virtual void printNavState(int state);

  private:            
    actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *_mbfExePathClient;  
        
};

#endif // BT_MOWPATHFOLLOW_H
