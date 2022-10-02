#ifndef BT_GETFIRSTPOST_H
#define BT_GETFIRSTPOST_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class GetFirstPose : public BT::SyncActionNode
{
  public:    
    GetFirstPose(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)       
    {
    }
    
    static BT::PortsList providedPorts()
    {
         return{
                BT::InputPort<nav_msgs::Path>("path"),
                BT::OutputPort<geometry_msgs::PoseStamped>("pose_out")
               };
    }
  
    BT::NodeStatus tick() override;
  private:      
};

#endif // BT_GETFIRSTPOST_H
