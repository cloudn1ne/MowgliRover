

#ifndef BT_GENERATE2DPOSE_H
#define BT_GENERATE2DPOSE_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Transform.h>

class Generate2DPose : public BT::SyncActionNode
{
  public:    
    Generate2DPose(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)       
    {
    }
    
    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<float>("x"),
                BT::InputPort<float>("y"),
                BT::InputPort<float>("yaw"),
                BT::OutputPort<geometry_msgs::PoseStamped>("pose_out") 
              };
    }
  
    BT::NodeStatus tick() override;
  private:      
};


#endif // BT_GENERATE2DPOSE_H
