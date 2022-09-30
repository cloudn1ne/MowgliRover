#ifndef BT_FORLOOP_H
#define BT_FORLOOP_H

#include "ros/ros.h"
#include "behaviortree_cpp_v3/control_node.h"

namespace BT
{
/**
 * @brief ForLoop must have exactly 2 children.
 *
 *
 * The first child is the "action" that is executed continuously until the loop_variable reaches loop_var_until
 *
 * When the loop is finished the second child "finish action" is executed
 * 
 */

class ForLoop : public ControlNode
{
    
public:
    ForLoop(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
         return{ 
                BT::InputPort<int>("loop_var_start"),
                BT::InputPort<int>("increment_by"),
                BT::InputPort<int>("loop_var_until"),
                BT::OutputPort<int>("loop_var")
               };
    }

    virtual ~ForLoop() override = default;

    virtual void halt() override;

    virtual void resetLoopVar(void);

private:
    virtual BT::NodeStatus tick() override;
    int _loop_var_start;
    int _loop_var_until;
    int _increment_by;
    int _loop_var;               /* incremented/decremented internally */
    bool _initialized;           
    std::string _name;
};

}   // namespace BT

#endif // BT_FORLOOP_H