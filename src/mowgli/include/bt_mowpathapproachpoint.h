

#ifndef BT_MOWPATHAPPROACHPOINT_H
#define BT_MOWPATHAPPROACHPOINT_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"    
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Transform.h>


// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".

class MowPathApproachPoint : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    MowPathApproachPoint(const std::string& name, const BT::NodeConfiguration& config,               
               actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfMoveBaseClient               
               )
      : StatefulActionNode(name, config), 
      _mbfMoveBaseClient(mbfMoveBaseClient)      
    {        
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<std::string>("planner"),
                BT::InputPort<geometry_msgs::PoseStamped>("pose") 
              };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;

    virtual void printNavState(int state);
    
  private:        
    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *_mbfMoveBaseClient;
};

#endif // BT_MOWPATHAPPROACHPOINT_H
