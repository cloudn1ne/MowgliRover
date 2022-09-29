/*
 * Mowgli GPSCONTROL Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */
#include "bt_gpscontrol.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "mowgli_bt.h" // externs

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{    
    factory.registerNodeType<GpsControl>("GpsControl");
}


BT::NodeStatus GpsControl::tick()
{
    std::string inp_enable;
    if ( !getInput<std::string>("enable", inp_enable))
    {
        throw BT::RuntimeError("missing required input [enable]");
    }

    ROS_INFO_STREAM("[ GpsControl: STARTED ]. enable='"<< inp_enable << "'");
        
    callSetGpsControlSrv(std::stoi(inp_enable)); // extern

    _halt_requested.store(false);
    int count = 0;

    // Pretend that "computing" takes 250 milliseconds.
    // It is up to you to check periodically _halt_requested and interrupt
    // this tick() if it is true.
    while (!_halt_requested && count++ < 25)
    {
        std::this_thread::sleep_for( std::chrono::milliseconds(10) );
    }

    ROS_INFO_STREAM("[ GpsControl: FINISHED ]");
    return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

void GpsControl::halt()
{
    _halt_requested.store(true);
}