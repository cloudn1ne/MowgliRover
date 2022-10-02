/*
 * Mowgli ForLoop Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 * 
 * Arguments:
 * 
 *    <ForLoop 
 *           loop_var="{loopvar}"
 *           loop_var_start="<start>"
 *           loop_var_until="<until>"
 *           increment_by="<inc>" 
 *    />
 *      <FirstNode/>
 *      <SecondNode/>
 *    </ForLoop>
 * 
 * Description:
 * 
 *    As long as <FirstNode> returns SUCCESS it will be ticked until loop_var reaches loop_var_until.
 *    loop_var must be a blackboard reference, which can be used within the loop to iterate over an array for example.
 *    The amount of increment is set via increment_by
 *
 *    The the loop is completed (or prematurely exited) the <SecondNode> will be ticked.
 * 
 */

#include "bt_forloop.h"


// #define BT_DEBUG 1

namespace BT
{

ForLoop::ForLoop(const std::string& name, const BT::NodeConfiguration& config) :
  ControlNode::ControlNode(name, config)
{
    setRegistrationID("ForLoop");    
    _initialized = false;
    _name = name;
}


void ForLoop::halt()
{
    ControlNode::halt();
}

NodeStatus ForLoop::tick()
{
    const size_t children_count = children_nodes_.size();

    if (children_count != 2)
    {
        throw std::logic_error("ForLoop must have 2 children");
    }  
    getInput("loop_var_until", _loop_var_until);
    getInput("increment_by", _increment_by);

    // if we are not initialized we need to return RUNNING, otherwise we can not propagate 
    // the setOutput() to our loop nodes
    if (!_initialized)
    {
        getInput("loop_var_start", _loop_var_start);

        // check basic constraints
        if (_loop_var_start >= _loop_var_until)
        {
            throw RuntimeError("ForLoop: loop_var_start needs to be lower then loop_var_until");
        }

        _loop_var = _loop_var_start;
        setOutput("loop_var", _loop_var);
        _initialized = true;
        return BT::NodeStatus::RUNNING;
    }
    

#ifdef BT_DEBUG   
    ROS_INFO_STREAM("===============================================");
    ROS_INFO_STREAM("ForLoop(" << _name << ") loop_var = " << _loop_var);
    ROS_INFO_STREAM("ForLoop(" << _name << ") loop_var_until = " << _loop_var_until);
    ROS_INFO_STREAM("ForLoop(" << _name << ") increment_by = " << _increment_by);
#endif

    setStatus(NodeStatus::RUNNING);
    NodeStatus status = NodeStatus::IDLE;    
    if (_loop_var < _loop_var_until)
    {               
        haltChild(1);   
#ifdef BT_DEBUG                
        ROS_INFO_STREAM("[ ForLoop(" << _name << "): loop_var = " << _loop_var << ", ticking child ]");
#endif        
        status = children_nodes_[0]->executeTick();          // ForLoop running - tick 1st child (whatever is in the loop)
        
        switch (status)
        {
            case BT::NodeStatus::SUCCESS:   // child Node status was SUCCESS so we inc our loopvar    
                                            _loop_var += _increment_by;
                                            setOutput("loop_var", _loop_var);
#ifdef BT_DEBUG                   
                                            ROS_INFO_STREAM("[ ForLoop(" << _name << "): incrementing loop_var because child status was "  << status << " ]");
#endif    
                                            return BT::NodeStatus::RUNNING;


            case  BT::NodeStatus::RUNNING:  // child Node status was RUNNING, so lets tick it again
#ifdef BT_DEBUG                   
                                            ROS_INFO_STREAM("[ ForLoop(" << _name << "): continueing loop because child status was "  << status << " ]");
#endif                                                
                                            return BT::NodeStatus::RUNNING;


            default:                        // child Node most likely reported FAILURE, in any case we abort the loop and tick the loop finished node
#ifdef BT_DEBUG                   
                                            ROS_INFO_STREAM("[ ForLoop(" << _name << "): aborted child status was " << status << " ]");
#endif                    
                                            resetLoopVar(); // reset _loop_var for next run
                                            status = children_nodes_[1]->executeTick();
                                            return status;

        }
    }   // _loop_var < _loop_var_until
    else
    {
#ifdef BT_DEBUG                   
                                            ROS_INFO_STREAM("[ ForLoop(" << _name << "): reached end of loop ]");
#endif                    
                                            resetLoopVar(); // reset _loop_var for next run
                                            status = children_nodes_[1]->executeTick();
                                            return status;        
    }
}

void ForLoop::resetLoopVar(void)
{
    getInput("loop_var_start", _loop_var_start);
    _loop_var = _loop_var_start;
    setOutput("loop_var", _loop_var);    
#ifdef BT_DEBUG           
    ROS_INFO_STREAM("[ ForLoop(" << _name << "): _loop_var reset to " << _loop_var << " ]");
#endif  

}


}   // namespace BT