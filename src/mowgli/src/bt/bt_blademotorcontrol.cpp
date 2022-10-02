/*
 * Mowgli BladeMotorControl Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 * 
 * Arguments:
 * 
 *    <BladeMotorControl enable="1"/>
 * 
 * Description:
 * 
 *    Enable (1) or disable (0) the blade motor.
 * 
 *    For this node to return SUCCESS, the blade motor feedback via the status topic needs to report an actually spinning blade motor
 * 
 */
#include "bt_blademotorcontrol.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#define BT_DEBUG 1


BT::NodeStatus BladeMotorControl::onStart()
{
      bool inp_enable;
      getInput("enable", inp_enable);
#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ BladeMotorControl: STARTED ]. enable='"<< inp_enable << "'");
#endif
      requested_state = inp_enable;              
      if( requested_state == *_blade_motor_enabled ) {
        // No need to go into the RUNNING state, or do anything
#ifdef BT_DEBUG                
        ROS_INFO_STREAM("[ BladeMotorControl: SUCCESS (already in requested_state)]");
#endif        
        return BT::NodeStatus::SUCCESS;
      }
      else 
      {
        callSvc(requested_state);        
        return BT::NodeStatus::RUNNING;
      }
}

// method invoked by an action in the RUNNING state.
BT::NodeStatus BladeMotorControl::onRunning()
{
      if( requested_state == *_blade_motor_enabled ) {
#ifdef BT_DEBUG                
        ROS_INFO_STREAM("[ BladeMotorControl: SUCCESS ]");
#endif        
        return BT::NodeStatus::SUCCESS;
      }
      else {
#ifdef BT_DEBUG                
        ROS_INFO_STREAM("[ BladeMotorControl: RUNNING ]");
#endif        
        return BT::NodeStatus::RUNNING;
      }
}

void BladeMotorControl::onHalted() 
{
      // nothing to do here...
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ BladeMotorControl: interrupted ]");    
#endif      
}

void BladeMotorControl::callSvc(bool requested_state)
{
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ BladeMotorControl: callSvc(" << requested_state << ") ]");    
#endif      
      mower_msgs::MowerControlSrv srv;
      srv.request.mow_enabled = requested_state;      
      _svcClient.call(srv);  
}
