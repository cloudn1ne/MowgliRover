#ifndef BT_STATUS_H
#define BT_STATUS_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"


// This is an asynchronous operation that will run in a separate thread.
class GetMowerBatteryVoltage : public BT::SyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    GetMowerBatteryVoltage(const std::string& name, const BT::NodeConfiguration& config,
                    float *state_v_battery )
      : SyncActionNode(name, config),
      _state_v_battery(state_v_battery)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::OutputPort<float>("out")};
    }

    BT::NodeStatus tick() override;
  private:      
    float *_state_v_battery;
};


class GetMowerBladeState : public BT::SyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    GetMowerBladeState(const std::string& name, const BT::NodeConfiguration& config,
                       bool *blade_motor_enabled )
      : SyncActionNode(name, config),
      _blade_motor_enabled(blade_motor_enabled)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::OutputPort<bool>("out") };
    }

    BT::NodeStatus tick() override;
  private:      
    bool *_blade_motor_enabled;
};

#endif // BT_STATUS_H
