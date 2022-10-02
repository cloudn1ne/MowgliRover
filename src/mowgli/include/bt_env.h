#ifndef BT_ENV_H
#define BT_ENV_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

class GetEnvString : public BT::SyncActionNode
{
  public:    
    GetEnvString(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)      
    {
    }
  
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<std::string>("var"),
                BT::OutputPort<std::string>("str_out") 
              };
    }

    BT::NodeStatus tick() override;
  private:      
    
};

class GetEnvInt : public BT::SyncActionNode
{
  public:    
    GetEnvInt(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)      
    {
    }
  
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<std::string>("var"),
                BT::OutputPort<int>("int_out") 
              };
    }

    BT::NodeStatus tick() override;
  private:      
    
};

class GetEnvFloat : public BT::SyncActionNode
{
  public:    
    GetEnvFloat(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)      
    {
    }
 
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<std::string>("var"),
                BT::OutputPort<float>("float_out") 
              };
    }

    BT::NodeStatus tick() override;
  private:      
    
};

class GetEnvBool : public BT::SyncActionNode
{
  public:    
    GetEnvBool(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)      
    {
    }

    
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<std::string>("var"),
                BT::OutputPort<bool>("bool_out") 
              };
    }

    BT::NodeStatus tick() override;
  private:      
    
};



class GetEnvBoolAsInt : public BT::SyncActionNode
{
  public:    
    GetEnvBoolAsInt(const std::string& name, const BT::NodeConfiguration& config)                       
      : SyncActionNode(name, config)      
    {
    }

    // It is mandatory to define this static method.
  
    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<std::string>("var"),
                BT::OutputPort<int>("int_out") 
              };
    }

    BT::NodeStatus tick() override;
  private:      
    
};

#endif // BT_ENV_H