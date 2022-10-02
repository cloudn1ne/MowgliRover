//
// included by all BT action Nodes  
//
#ifndef BT_NODES_H
#define BT_NODES_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mbf_msgs/ExePathAction.h"
#include "actionlib/client/simple_action_client.h"



namespace DummyNodes
{

using BT::NodeStatus;


// Example of custom SyncActionNode (synchronous action)
// with an input port.
class SaySomething : public BT::SyncActionNode
{
  public:
    SaySomething(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }
};

// Example of custom SyncActionNode (synchronous action)
// with an input port.
class SayString : public BT::SyncActionNode
{
  public:
    SayString(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }
};


// Example of custom SyncActionNode (synchronous action)
// with an input port.
class SayFloat : public BT::SyncActionNode
{
  public:
    SayFloat(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<float>("message") };
    }
};

// Example of custom SyncActionNode (synchronous action)
// with an input port.
class SayInt : public BT::SyncActionNode
{
  public:
    SayInt(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<int>("message") };
    }
};


// Example of custom SyncActionNode (synchronous action)
// with an input port.
class SayBool : public BT::SyncActionNode
{
  public:
    SayBool(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<bool>("message") };
    }
};



// Example os Asynchronous node that use StatefulActionNode as base class
class SleepNode : public BT::StatefulActionNode
{
  public:
    SleepNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return{ BT::InputPort<int>("msec") };
    }

    NodeStatus onStart() override
    {
        int msec = 0;
        getInput("msec", msec);
        if( msec <= 0 )
        {
            // no need to go into the RUNNING state
            return NodeStatus::SUCCESS;
        }
        else {
            using namespace std::chrono;
            // once the deadline is reached, we will return SUCCESS.
            deadline_ = system_clock::now() + milliseconds(msec);
            return NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    NodeStatus onRunning() override
    {
        if ( std::chrono::system_clock::now() >= deadline_ )
        {
            return NodeStatus::SUCCESS;
        }
        else {
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        // nothing to do here...
        std::cout << "SleepNode interrupted" << std::endl;
    }

  private:
    std::chrono::system_clock::time_point deadline_;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{   
    factory.registerNodeType<SaySomething>("SaySomething");
}

} // end namespace

#endif   // BT_NODES_H