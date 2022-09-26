#include "bt_mowcontrol.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// msg 
#include "mower_msgs/MowerControlSrv.h"

// #define BT_DEBUG 1


BT::NodeStatus MowControl::onStart()
{
      bool inp_enable;
      getInput("enable", inp_enable);
#ifdef BT_DEBUG        
      ROS_INFO_STREAM("[ MowControl: STARTED ]. enable='"<< inp_enable << "'");
#endif
      requested_state = inp_enable;              
      if( requested_state == *_blade_motor_enabled ) {
        // No need to go into the RUNNING state, or do anything
#ifdef BT_DEBUG                
        ROS_INFO_STREAM("[ MowControl: SUCCESS (already in requested_state)]");
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
BT::NodeStatus MowControl::onRunning()
{
      if( requested_state == *_blade_motor_enabled ) {
#ifdef BT_DEBUG                
        ROS_INFO_STREAM("[ MowControl: SUCCESS ]");
#endif        
        return BT::NodeStatus::SUCCESS;
      }
      else {
#ifdef BT_DEBUG                
        ROS_INFO_STREAM("[ MowControl: RUNNING ]");
#endif        
        return BT::NodeStatus::RUNNING;
      }
}

void MowControl::onHalted() 
{
      // nothing to do here...
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ MowControl: interrupted ]");    
#endif      
}

void MowControl::callSvc(bool requested_state)
{
#ifdef BT_DEBUG              
      ROS_INFO_STREAM("[ MowControl: callSvc(" << requested_state << ") ]");    
#endif      
      mower_msgs::MowerControlSrv srv;
      srv.request.mow_enabled = requested_state;      
      _svcClient.call(srv);  
}
