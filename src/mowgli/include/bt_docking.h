

#ifndef BT_DOCKING_H
#define BT_DOCKING_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mbf_msgs/ExePathAction.h"
#include "actionlib/client/simple_action_client.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "mower_map/GetDockingPointSrv.h"

extern void stopMoving();

// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".

class Docking : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    Docking(const std::string& name, const BT::NodeConfiguration& config,
               actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClient,
               ros::ServiceClient svcClient,
               float *v_charge
               )
      : StatefulActionNode(name, config),
      _mbfClient(mbfClient),
      _svcClient(svcClient),
      _v_charge(v_charge)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<float>("docking_distance"),
                BT::InputPort<std::string>("planner") 
              };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;

    virtual void printNavState(int state);

    virtual geometry_msgs::PoseStamped getDockingPose();

    virtual bool isDocked();

  private:    
    // uint8_t requested_state;
    ros::ServiceClient _svcClient;
    actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *_mbfClient;
    float *_v_charge;
};

#endif // BT_DOCKING_H
