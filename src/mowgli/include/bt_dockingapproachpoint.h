

#ifndef BT_DOCKINGAPPROACHPointPOINT_H
#define BT_DOCKINGAPPROACHPointPOINT_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"    
#include "actionlib/client/simple_action_client.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Transform.h>
#include "mower_map/GetDockingPointSrv.h"


// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".

class DockingApproachPoint : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    DockingApproachPoint(const std::string& name, const BT::NodeConfiguration& config,               
               actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfMoveBaseClient,
               ros::ServiceClient svcClient    
               )
      : StatefulActionNode(name, config), 
      _mbfMoveBaseClient(mbfMoveBaseClient),
      _svcClient(svcClient)
    {        
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<float>("docking_approach_distance"),
                BT::InputPort<std::string>("planner") 
              };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;

    virtual void printNavState(int state);

    virtual geometry_msgs::PoseStamped getDockingPose();

  private:        
    ros::ServiceClient _svcClient;    
    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *_mbfMoveBaseClient;
};

#endif // BT_DOCKINGAPPROACHPointPOINT_H
