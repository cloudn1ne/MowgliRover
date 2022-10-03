/*
 * Mowgli Generate2DPose Nodes V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 * Arguments:
 * 
 *    <Generate2DPose x={x} y={y} yaw={yaw}  pose_pose="{pose}"/>
 * 
 * Description:
 * 
 *    Generate a {pose} based on X,Y coordinates and a YAW (in degrees)
 *    The generated pose can be used with <ApproachPose/>
 * 
 *    Always returns success.
 */

#include "bt_generate2dpose.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1

/// @brief return current battery voltage
/// @return float "out" port
BT::NodeStatus Generate2DPose::tick()
{    
     geometry_msgs::PoseStamped convertedPose;
     float x, y, yaw;
     
     getInput("x", x);
     getInput("y", y);
     getInput("yaw", yaw);

#ifdef BT_DEBUG    
    ROS_INFO_STREAM("mowgli_bt: Generate2DPose::tick() x = " << x << "m / y = " << y << "m / yaw = " << yaw << "deg");
#endif    

    tf2::Quaternion convertedPoseQuaternion;
    convertedPoseQuaternion.setRPY( 0, 0, yaw*M_PI/180 );      // radians !

    convertedPose.header.frame_id = "map";
    convertedPose.header.stamp = ros::Time::now();   
    convertedPose.pose.position.x = x; 
    convertedPose.pose.position.y = y;
    convertedPose.pose.orientation = tf2::toMsg(convertedPoseQuaternion);

    setOutput("pose_out", convertedPose);    
    return BT::NodeStatus::SUCCESS;
}
