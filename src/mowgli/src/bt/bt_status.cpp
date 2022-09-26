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