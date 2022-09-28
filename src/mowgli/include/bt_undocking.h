

#ifndef BT_UNDOCKING_H
#define BT_UNDOCKING_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mbf_msgs/ExePathAction.h"
#include "actionlib/client/simple_action_client.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Transform.h>


// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".

class Undocking : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    Undocking(const std::string& name, const BT::NodeConfiguration& config,
               actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClient,
               nav_msgs::Odometry odom               
               )
      : StatefulActionNode(name, config),
      _mbfClient(mbfClient),
      _odom(odom)      
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<float>("undock_distance") };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;

    virtual void printNavState(int state);

  private:    
    // uint8_t requested_state;
    ros::ServiceClient _svcClient;
    actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *_mbfClient;
    nav_msgs::Odometry _odom;
};

#endif // BT_UNDOCKING_H
