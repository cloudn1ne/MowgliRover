/*
 * Mowgli TrimPoses Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */
#include "bt_trimposes.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1

/// @brief  extract the first pose from a given <mowpath>
/// @return writes <pose_out>
BT::NodeStatus TrimPoses::tick()
{    
#ifdef BT_DEBUG    
      ROS_INFO("mowgli_bt: TrimPoses::tick()");
#endif       
      nav_msgs::Path mowPath;
      getInput("mowpath", mowPath);
      auto &poses = mowPath.poses;

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
      }

#ifdef BT_DEBUG          
      ROS_INFO_STREAM("mowgli_bt: TrimPoses() mowpath has " << (poses.size()) << " poses, trimming " << (end-begin) << " poses, beginning at " << begin);
#endif  
      // remove poses from mowPath
      poses.erase(poses.begin() + begin , poses.begin() + end);
      if (poses.size())
      {
            setOutput("mowPath", mowPath);    
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: TrimPoses() trim complete there are " << (poses.size()-1) << " poses left");
#endif                         
            return BT::NodeStatus::SUCCESS;
      }
      else
      {
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: TrimPoses() there are no poses left in the mowpath");
#endif             
            return BT::NodeStatus::FAILURE;
      }
      
}
