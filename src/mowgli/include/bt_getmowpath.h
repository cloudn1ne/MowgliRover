#ifndef BT_GETMOWPATH_H
#define BT_GETMOWPATH_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "nav_msgs/Path.h"

class GetMowPath : public BT::SyncActionNode
{
  public:    
    GetMowPath(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)       
    {
    }
    
    static BT::PortsList providedPorts()
    {
         return{ 
                BT::InputPort<int>("path_index"),
                BT::InputPort<std::vector<slic3r_coverage_planner::Path>>("mowplan"),
                BT::OutputPort<nav_msgs::Path>("mowpath_out")
               };
    }
  
    BT::NodeStatus tick() override;
  private:      
    ros::ServiceClient _svcMapClient;
    ros::ServiceClient _svcPlannerGetProgressClient;
};

#endif // BT_GETMOWPATH_H
