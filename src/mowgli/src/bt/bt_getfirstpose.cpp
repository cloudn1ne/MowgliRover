/*
 * Mowgli GetFirstPose Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */
#include "bt_getfirstpose.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1

/// @brief  extract the first pose from a given <mowpath>
/// @return writes <pose_out>
BT::NodeStatus GetFirstPose::tick()
{    
#ifdef BT_DEBUG    
      ROS_INFO("mowgli_bt: GetFirstPose::tick()");
#endif       
      nav_msgs::Path mowPath;
      getInput("mowpath", mowPath);

      auto &poses = mowPath.poses;

#ifdef BT_DEBUG          
      ROS_INFO_STREAM("mowgli_bt: GetFirstPose() mowpath has " << (poses.size()) << " poses");
#endif  
      if (poses.size() >= 1)
      {
            setOutput("pose_out", poses[0]);
#ifdef BT_DEBUG       
            // Calculate yaw (only for display purposes)
            tf2::Quaternion quat;
            tf2::fromMsg(poses[0].pose.orientation, quat);
            tf2::Matrix3x3 m(quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);      
            ROS_INFO_STREAM("[ mowgli_bt: GetFirstPose() ] pose = x = "<< poses[0].pose.position.x << " y= " << poses[0].pose.position.y << " yaw = " << yaw*180/M_PI << " deg");
#endif            
            return BT::NodeStatus::SUCCESS;
      }
      else
      {
#ifdef BT_DEBUG          
            ROS_INFO_STREAM("mowgli_bt: GetFirstPose() there are no poses left in the mowpath");
#endif             
            return BT::NodeStatus::FAILURE;
      }
      
}
