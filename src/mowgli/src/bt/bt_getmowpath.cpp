/*
 * Mowgli GetMowPath Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */
#include "bt_getmowpath.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1

/// @brief return current battery voltage
/// @return float "out" port
BT::NodeStatus GetMowPath::tick()
{    
#ifdef BT_DEBUG    
      ROS_INFO("mowgli_bt: GetMowPath::tick()");
#endif    
      int path_index = -1;
      getInput("path_index", path_index);

      std::vector<slic3r_coverage_planner::Path> mowPlan;
      getInput("mowplan", mowPlan);

#ifdef BT_DEBUG          
      ROS_INFO_STREAM("mowgli_bt: GetMowPath() mowpath[" << path_index << "/" << (mowPlan.size()-1) << "] has " << (mowPlan[path_index].path.poses.size()) << " poses");
#endif  
      if (path_index < mowPlan.size())
      {
            setOutput("mowpath_out", mowPlan[path_index].path); 
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: GetMowPath() mowpath_out updated");
#endif              
            return BT::NodeStatus::SUCCESS;
      }
      else
      {
            return BT::NodeStatus::FAILURE;
      }
      
}
