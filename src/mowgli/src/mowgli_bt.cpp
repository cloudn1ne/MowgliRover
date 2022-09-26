/*
 * Mowgli BehaviorTree V 1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 *
 * v1.0: initial release
 *
 * needs: sudo apt-get install ros-noetic-behaviortree-cpp-v3
 * 
 */

#include "ros/ros.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"       // ZeroMQ for Groot
#include "signal.h"

// standard messages/srvs
#include "std_msgs/String.h"

// non standard messages/srvs
#include "mower_msgs/GPSControlSrv.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mowgli/status.h"


#include "bt_nodes.h"
#include "bt_gpscontrol.h"
#include "bt_mowcontrol.h"
#include "bt_status.h"

#define SERVICE_CLIENT_WAIT_TIMEOUT     60.0        /* amount of time to wait for services to become available */

using namespace BT;

// global states used by nodes
bool gstate_blade_motor_enabled;
bool gstate_is_charging;
float gstate_v_battery;

// Subscribers
ros::Subscriber subMowgliStatus;            /* mowgli/status */

// Publishers
ros::Publisher pubCurrentState;             /* /mower_logic/current_state */

// Service clients
ros::ServiceClient srvGpsControlClient;     /* mower_service/set_gps_state */
ros::ServiceClient srvMowClient;            /* mower_service/mow_enabled */


/// @brief enable or disable the GPS receiver
/// @param enabled 
/// @return atm always true
bool callSetGpsControlSrv(bool enabled) 
{
    mower_msgs::GPSControlSrv srv;
    srv.request.gps_enabled = enabled;    
    srvGpsControlClient.call(srv);
    // TODO check result
    return true;
}

/// @brief signal handler (e.g. ctrl-c)
/// @param sig 
void mySigintHandler(int sig)
{ 
    ros::shutdown();
}

void publishCurrentState(const std::string& state_str)
{
    std_msgs::String state_name;
    state_name.data = state_str;
    pubCurrentState.publish(state_name);
}

/// @brief wait for all services required to become available
/// @param  
/// @return true if all are found, false if one is not found within a given timeout
bool waitForServices(void)
{
    ROS_INFO("mowgli_bt: Waiting for GpsControl service ...");
    if (!srvGpsControlClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for GpsControl service");        
        return false;
    }
    ROS_INFO("mowgli_bt: Waiting for Mow service ...");
    if (!srvMowClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for Mow service");        
        return false;
    }

    return true; // all services present
}


void MowgliStatusCB(const mowgli::status::ConstPtr &msg) 
{    
//    ROS_INFO_STREAM("mowgli_bt: MowgliStatusCB");

    gstate_blade_motor_enabled = msg->blade_motor_enabled;
    gstate_v_battery = msg->v_battery;
    gstate_is_charging = msg->is_charging;    
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mowgli_bt");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    signal(SIGINT, mySigintHandler);
    ROS_INFO_STREAM("mowgli_bt: Starting mowgli_bt");

    ros::AsyncSpinner asyncSpinner(1);  
    asyncSpinner.start();

    // Subscribers
    subMowgliStatus = n.subscribe("mowgli/status", 50, MowgliStatusCB);

    // Publishers
    pubCurrentState = n.advertise<std_msgs::String>("mower_logic/current_state", 10, true);
    
    // Service clients
    srvGpsControlClient = n.serviceClient<mower_msgs::GPSControlSrv>("mower_service/set_gps_state");
    srvMowClient =  n.serviceClient<mower_msgs::MowerControlSrv>("mower_service/mow_enabled");

    if (!waitForServices())
    {
        ros::shutdown(); // bye bye
        exit(-1);
    }
    ROS_INFO_STREAM("mowgli_bt: All services are now available");



    BehaviorTreeFactory factory;    
    using namespace DummyNodes;
    // The recommended way to create a Node is through inheritance.
    // Even if it requires more boilerplate, it allows you to use more functionalities
    // like ports (we will discuss this in future tutorials).
    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerNodeType<SaySomething>("SaySomething");
    // custom bt_nodes for debugging
    factory.registerNodeType<SayFloat>("SayFloat");
    factory.registerNodeType<SayInt>("SayInt");
    factory.registerNodeType<SayBool>("SayBool");

    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;

    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));

    // Mowgli Nodes
    // bt_gpscontrol
    factory.registerNodeType<GpsControl>("GpsControl");

    // bt_mowcontrol (MowControl)
    NodeBuilder builder_MowControl =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<MowControl>( name, config, srvMowClient, &gstate_blade_motor_enabled );
    };
    factory.registerBuilder<MowControl>( "MowControl", builder_MowControl);

    // bt_status (GetMowerBatteryVoltage)
    NodeBuilder builder_GetMowerBatteryVoltage =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetMowerBatteryVoltage>( name, config, &gstate_v_battery );
    };
    factory.registerBuilder<GetMowerBatteryVoltage>( "GetMowerBatteryVoltage", builder_GetMowerBatteryVoltage);

    // bt_status (GetMowerBladeState)
    NodeBuilder builder_GetMowerBladeState =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetMowerBladeState>( name, config, &gstate_blade_motor_enabled );
    };
    factory.registerBuilder<GetMowerBladeState>( "GetMowerBladeState", builder_GetMowerBladeState);




    // Load tree from XML
    auto tree = factory.createTreeFromFile("../MowgliRover/src/mowgli/src/tree.xml");
    PublisherZMQ publisher_zmq(tree); // for Groot

    ros::Rate r(1); // ROS loop rate
    while (ros::ok()) {
        r.sleep();        
        tree.tickRoot();
    }
    return 0;
}
