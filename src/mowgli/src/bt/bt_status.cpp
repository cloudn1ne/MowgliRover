/*
 * Mowgli STATUS Nodes V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */
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



/// @brief save current high level command to the blackboard
/// @return boolean "out" port
BT::NodeStatus GetHighLevelCommand::tick()
{   
    std::string inp_output_key;
    getInput("output_key", inp_output_key);

    if (*_highlevel_command > 0)
    {
#ifdef BT_DEBUG        
    ROS_INFO("mowgli_bt: GetHighLevelCommand::tick() -> SUCCESS");
#endif        
        setOutput("output_key", std::to_string(*_highlevel_command) );
        *_highlevel_command = 0;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
#ifdef BT_DEBUG        
    ROS_INFO("mowgli_bt: GetHighLevelCommand::tick() -> FAILURE");
#endif        
        setOutput("output_key", "NULL");
        return BT::NodeStatus::FAILURE;
    }
}
