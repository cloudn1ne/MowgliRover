

#ifndef BT_GPS_CONTROL_H
#define BT_GPS_CONTROL_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"


// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".

class GpsControl : public BT::AsyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    GpsControl(const std::string& name, const BT::NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("enable") };
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
};

#endif // BT_GPS_CONTROL
