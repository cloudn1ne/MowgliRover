/*
 * Mowgli GetMowPath Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 * 
 * Arguments:
 * 
 *    <GetMowPath path_index="{path_index}" mowplan="{mowplan}" path_out="{mowpath}"/>
 * 
 * Description:
 * 
 *    Extract the n-th (path_index) path out of a mowplan (array of paths) and save the extracted path into path_out
 * 
 *    For this node to return SUCCESS, the selected path_index must exist with the mowplan
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

      if (path_index < mowPlan.size())
      {
            setOutput("path_out", mowPlan[path_index].path); 
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: GetMowPath() mowpath[" << path_index << "/" << (mowPlan.size()-1) << "] has " << (mowPlan[path_index].path.poses.size()) << " poses");
            ROS_INFO_STREAM("mowgli_bt: GetMowPath() path_out updated");
#endif              
            return BT::NodeStatus::SUCCESS;
      }
      else
      {
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: GetMowPath() mowpath[" << path_index << "] unknown");
#endif  
            return BT::NodeStatus::FAILURE;
      }
      
}
