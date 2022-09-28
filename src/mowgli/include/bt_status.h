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

class IsCharging : public BT::SyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    IsCharging(const std::string& name, const BT::NodeConfiguration& config,
                       bool *is_charging )
      : SyncActionNode(name, config),
      _is_charging(is_charging)
    {
    }

    BT::NodeStatus tick() override;
  private:      
    bool *_is_charging;
};

class IsMowing : public BT::SyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    IsMowing(const std::string& name, const BT::NodeConfiguration& config,
                       bool *blade_motor_enabled )
      : SyncActionNode(name, config),
      _blade_motor_enabled(blade_motor_enabled)
    {
    }

    BT::NodeStatus tick() override;
  private:      
    bool *_blade_motor_enabled;
};

class GetHighLevelCommand : public BT::SyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    GetHighLevelCommand(const std::string& name, const BT::NodeConfiguration& config,
                       uint8_t *highlevel_command )
      : SyncActionNode(name, config),
      _highlevel_command(highlevel_command)
    {
    }

    // It is mandatory to define this static method.
  
    static BT::PortsList providedPorts()
    {
        return{ BT::BidirectionalPort<std::string>("output_key") };
    }


    BT::NodeStatus tick() override;
  private:      
    uint8_t *_highlevel_command;
};

#endif // BT_STATUS_H
