#include "bt_status.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// #define BT_DEBUG 1

/// @brief return current battery voltage
/// @return float "out" port
BT::NodeStatus GetMowerBatteryVoltage::tick()
{    
#ifdef BT_DEBUG    
    ROS_INFO("mowgli_bt: GetMowerBatteryVoltage::tick()");
#endif    
    setOutput("out", *_state_v_battery );    
    return BT::NodeStatus::SUCCESS;
}


/// @brief return state of blade motor
/// @return boolean "out" port
BT::NodeStatus GetMowerBladeState::tick()
{    
#ifdef BT_DEBUG        
    ROS_INFO("mowgli_bt: GetMowerBladeState::tick()");
#endif    
    setOutput("out", *_blade_motor_enabled );    
    return BT::NodeStatus::SUCCESS;
}


/// @brief return charging state
/// @return boolean "out" port
BT::NodeStatus IsCharging::tick()
{          
    if (*_is_charging)
    {
#ifdef BT_DEBUG        
    ROS_INFO("mowgli_bt: IsCharging::tick() -> SUCCESS");
#endif                
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
#ifdef BT_DEBUG        
    ROS_INFO("mowgli_bt: IsCharging::tick() -> SUCCESS");
#endif                        
        return BT::NodeStatus::FAILURE;
    }
}

/// @brief return charging state
/// @return boolean "out" port
BT::NodeStatus IsMowing::tick()
{   
    

    if (*_blade_motor_enabled)
    {
#ifdef BT_DEBUG        
    ROS_INFO("mowgli_bt: IsMowing::tick() -> SUCCESS");
#endif        
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
#ifdef BT_DEBUG        
    ROS_INFO("mowgli_bt: IsMowing::tick() -> FAILURE");
#endif        
        return BT::NodeStatus::FAILURE;
    }
}
