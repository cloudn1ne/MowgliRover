/*
 * Mowgli Status Nodes V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 * Arguments:
 * 
 *    <GetMowerBatteryVoltage out="{voltage}"/>
 * 
 * Description:
 * 
 *    Get current battery voltage and store to {voltage} variable
 * 
 *    Always returns success.
 * 
 * ------------------------------------------------------------------------------------
 * 
* Arguments:
 * 
 *    <GetHighLevelCommand cmd_out="{hlc}"/>
 * 
 * Description:
 * 
 *    Get current value of high level command topic and write it to cmd_out (as a string).
 *    You can use that string variable with something like the SwitchX statement.
 * 
 *    Returns SUCCESS if the hlc received is >0, otherwise FAILURE is returned
 * 
 * ------------------------------------------------------------------------------------
 *  
 * <WaitForOdom>
 * 
 *  Description
 *      Returns RUNNING while waiting for /odom to become valid again, SUCCESSFULL when valid again.
 * 
 *      Note: use a wrapping <Timeout> node to handle timeout waiting for /odom, as this node will not provide a timeout mechanism. 
 *            e.g. it will never return FAILURE
 *
 * ------------------------------------------------------------------------------------
 * 
 * <IsCharging/>, <isMowing/>, <isOdomValid/> return SUCCESS or FAILURE depending on the state of the mower
 * 
 */
#include "bt_status.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1

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
    ROS_INFO("mowgli_bt: IsCharging::tick() -> FAILURE");
#endif                        
        return BT::NodeStatus::FAILURE;
    }
}

/// @brief return blade motor state
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
///        <GetHighLevelCommand cmd_out="{high_level_command}"/>
/// @return string "cmd_out" port (convert the integer of the HLC ros topic to a string to be compatible with the SwitchX node)
BT::NodeStatus GetHighLevelCommand::tick()
{   
    std::string inp_output_key;
    getInput("cmd_out", inp_output_key);

    if (*_highlevel_command > 0)
    {
#ifdef BT_DEBUG        
    ROS_INFO("mowgli_bt: GetHighLevelCommand::tick() -> SUCCESS");
#endif        
        setOutput("cmd_out", std::to_string(*_highlevel_command) );
        *_highlevel_command = 0;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {       
        setOutput("cmd_out", "NULL");
        return BT::NodeStatus::FAILURE;
    }
}


/// @brief return state of odometry topic
/// @return SUCCESS/FAILURE
BT::NodeStatus IsOdomValid::tick()
{           
    // return node status
    if (*_odom_valid)
    {
#ifdef BT_DEBUG        
        ROS_INFO("mowgli_bt: IsOdomValid::tick() -> SUCCESS");
#endif 
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
#ifdef BT_DEBUG        
        ROS_INFO("mowgli_bt: IsOdomValid::tick() -> FAILURE");
#endif 
        return BT::NodeStatus::FAILURE;
    }
}


/// @brief Wait for /odom to be valid
/// @return RUNNING while waiting for /odom to become valid again, RUNNING when valid again - use a wrapping <Timeout> node to handle timeout waiting for /odom, as this node will not provide a timeout mechanism 
BT::NodeStatus WaitForOdom::onStart()
{
    if (*_odom_valid)
    {
#ifdef BT_DEBUG        
        ROS_INFO_STREAM("[ WaitForOdom: SUCCESS ]");
#endif
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
#ifdef BT_DEBUG        
        ROS_INFO_STREAM("[ WaitForOdom: RUNNING ]");
#endif
        return BT::NodeStatus::RUNNING;
    }
}

/// @brief Monitor the current MBF Goal Execution
BT::NodeStatus WaitForOdom::onRunning()
{
   if (*_odom_valid)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}

void WaitForOdom::onHalted() 
{
#ifdef BT_DEBUG              
    ROS_INFO_STREAM("[ WaitForOdom: interrupted - FAILURE ]");    
#endif      
}

