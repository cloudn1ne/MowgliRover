/*
 * Mowgli TrimPoses Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 * 
 * Arguments:
 * 
 *    <TrimPoses path="{mowpath}" begin="0" end="{mowpath_index}"/>
 * 
 * Description:
 * 
 *    Remove poses from a path. 
 *  
 *    For this node to return SUCCESS if the removal of poses was successful
 * 
 */
#include "bt_trimposes.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1

/// @brief  remove poses from <path>
/// @return writes <path>
BT::NodeStatus TrimPoses::tick()
{    
#ifdef BT_DEBUG    
      ROS_INFO("mowgli_bt: TrimPoses::tick()");
#endif       
      nav_msgs::Path path;
      getInput("path", path);
      auto &poses = path.poses;

      int begin = -1;
      int end = -1;
      getInput("begin", begin);
      getInput("end", end);

      if (end < begin)
      {
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: TrimPoses() end (" << end <<  ") must be greater than begin (" << begin << ")");
#endif  
            return BT::NodeStatus::FAILURE;
      }
      if ((begin > poses.size()-1) || (end > poses.size()-1))
      {
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: TrimPoses() end (" << end <<  ") and begin (" << begin << ") must be less than available poses (" << (poses.size()-1) << ")");
#endif              
            return BT::NodeStatus::FAILURE;
      }

#ifdef BT_DEBUG          
      ROS_INFO_STREAM("mowgli_bt: TrimPoses() path has " << (poses.size()) << " poses, trimming " << (end-begin) << " poses, beginning at " << begin);
#endif  
      // remove poses from path
      poses.erase(poses.begin() + begin , poses.begin() + end);
      if (poses.size())
      {
            setOutput("path", path);    
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: TrimPoses() trim complete there are " << (poses.size()) << " poses left");
#endif                         
            return BT::NodeStatus::SUCCESS;
      }
      else
      {
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: TrimPoses() there are no poses left in the path");
#endif             
            return BT::NodeStatus::FAILURE;
      }
      
}
