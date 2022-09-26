

#ifndef BT_MOW_CONTROL_H
#define BT_MOW_CONTROL_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"


// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".

class MowControl : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    MowControl(const std::string& name, const BT::NodeConfiguration& config,
               ros::ServiceClient svcClient, bool *blade_motor_enabled )
      : StatefulActionNode(name, config),
      _svcClient(svcClient), _blade_motor_enabled(blade_motor_enabled)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<bool>("enable") };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;

    virtual void callSvc(bool requested_state);

  private:    
    uint8_t requested_state;
    ros::ServiceClient _svcClient;
    bool *_blade_motor_enabled;
};

#endif // BT_MOW_CONTROL
