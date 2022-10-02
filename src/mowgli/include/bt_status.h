#ifndef BT_STATUS_H
#define BT_STATUS_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

class GetMowerBatteryVoltage : public BT::SyncActionNode
{
  public:    
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
    GetHighLevelCommand(const std::string& name, const BT::NodeConfiguration& config,
                       uint8_t *highlevel_command )
      : SyncActionNode(name, config),
      _highlevel_command(highlevel_command)
    {
    }

    // It is mandatory to define this static method.
  
    static BT::PortsList providedPorts()
    {
        return{ BT::BidirectionalPort<std::string>("cmd_out") };
    }


    BT::NodeStatus tick() override;
  private:      
    uint8_t *_highlevel_command;
};


/// @brief return state of /odom topic
class IsOdomValid : public BT::SyncActionNode
{
  public:
    
    IsOdomValid(const std::string& name, const BT::NodeConfiguration& config,
                       bool *odom_valid )
      : SyncActionNode(name, config),
      _odom_valid(odom_valid)
    {
    
    }

    BT::NodeStatus tick() override;

  private:      
    bool *_odom_valid;
    
};


class WaitForOdom : public BT::StatefulActionNode
{
  public:
    WaitForOdom(const std::string& name, const BT::NodeConfiguration& config,
               bool *odom_valid )         
      : StatefulActionNode(name, config),
      _odom_valid(odom_valid)
    {
        
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;
    
  private:          
    bool *_odom_valid;
};



#endif // BT_STATUS_H
