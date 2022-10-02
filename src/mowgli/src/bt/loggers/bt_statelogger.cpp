/*
 * Mowgli StateLogger V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 * Emits a ROS Topic thats contains the currently active node name
 *  
 */

#include "ros/ros.h"
#include "loggers/bt_statelogger.h"
#include "behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h"

namespace BT
{

StateLogger::StateLogger(const BT::Tree& tree, ros::Publisher pubCurrentState, uint16_t buffer_size):
  StatusChangeLogger(tree.rootNode()), buffer_max_size_(buffer_size)
{  
    enableTransitionToIdle(true);
   
    _pubCurrentState = pubCurrentState;
}

StateLogger::~StateLogger()
{
    // nothing
}

void StateLogger::callback(Duration timestamp, const TreeNode& node,
                          NodeStatus prev_status, NodeStatus status)
{
    if (status == BT::NodeStatus::RUNNING)
    {
#ifdef BT_DEBUG      
      ROS_INFO_STREAM("StateLogger::callback active node: " << node.registrationName() << "/" << node.name());
#endif      
      std_msgs::String state_name;
      state_name.data = node.registrationName() + "/" + node.name();
      _pubCurrentState.publish(state_name);    
    }
}

void StateLogger::flush()
{
    // nothing
}

}   // namespace BT