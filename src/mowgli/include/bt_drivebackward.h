

#ifndef BT_DRIVEBACKWARD_H
#define BT_DRIVEBACKWARD_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mbf_msgs/ExePathAction.h"
#include "actionlib/client/simple_action_client.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"


class DriveBackward : public BT::StatefulActionNode
{
  public:    
    DriveBackward(const std::string& name, const BT::NodeConfiguration& config,
               actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfExePathClient,
               nav_msgs::Odometry *odom               
               )
      : StatefulActionNode(name, config),
      _mbfExePathClient(mbfExePathClient),
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

#endif // BT_DRIVEBACKWARD_H
