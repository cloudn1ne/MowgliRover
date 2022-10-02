

#ifndef BT_DRIVEFORWARD_H
#define BT_DRIVEFORWARD_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mbf_msgs/ExePathAction.h"
#include "actionlib/client/simple_action_client.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"


class DriveForward : public BT::StatefulActionNode
{
  public:    
    DriveForward(const std::string& name, const BT::NodeConfiguration& config,
               actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *_mbfExePathClient,
               nav_msgs::Odometry *odom               
               )
      : StatefulActionNode(name, config),
      _mbfExePathClient(_mbfExePathClient),
      _odom(odom)      
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<float>("distance"), 
                BT::InputPort<std::string>("planner") 
              };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    virtual void onHalted() override;

    virtual void printNavState(int state);

  private:        
    ros::ServiceClient _svcClient;
    actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *_mbfExePathClient;
    nav_msgs::Odometry *_odom;
};

#endif // BT_DRIVEFORWARD_H
