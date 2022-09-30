#ifndef BT_GETMOWPLAN_H
#define BT_GETMOWPLAN_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Path.h"

class GetMowPlan : public BT::SyncActionNode
{
  public:    
    GetMowPlan(const std::string& name, const BT::NodeConfiguration& config,
                        ros::ServiceClient svcMapClient,
                        ros::ServiceClient svcPlannerGetProgressClient
                       )
      : SyncActionNode(name, config),
       _svcMapClient(svcMapClient),
       _svcPlannerGetProgressClient(svcPlannerGetProgressClient)
    {
    }
  
    static BT::PortsList providedPorts()
    {
         return{ 
                BT::InputPort<int>("area_index") ,
                BT::OutputPort<std::vector<slic3r_coverage_planner::Path>>("mowplan")
               };
    }
  
    BT::NodeStatus tick() override;
  private:      
    ros::ServiceClient _svcMapClient;
    ros::ServiceClient _svcPlannerGetProgressClient;
};

#endif // BT_GETMOWPLAN_H
