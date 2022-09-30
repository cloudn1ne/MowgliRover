#ifndef BT_TRIMPOSES_H
#define BT_TRIMPOSES_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class TrimPoses : public BT::SyncActionNode
{
  public:    
    TrimPoses(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)       
    {
    }
    
    static BT::PortsList providedPorts()
    {
         return{
                BT::InputPort<int>("begin"),
                BT::InputPort<int>("end"),
                BT::BidirectionalPort<nav_msgs::Path>("mowpath")                
               };
    }
  
    BT::NodeStatus tick() override;
  private:      
};

#endif // BT_TRIMPOSES_H
