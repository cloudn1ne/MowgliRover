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

// #define BT_DEBUG 1

// OS
#include "signal.h"

// BT

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"       // ZeroMQ for Groot

// ROS
#include "ros/ros.h"
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

// standard messages/srvs
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"

// non standard messages/srvs
#include "mower_msgs/GPSControlSrv.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mowgli/status.h"



#include "bt_nodes.h"
#include "bt_gpscontrol.h"
#include "bt_mowcontrol.h"
#include "bt_dockingapproach.h"
#include "bt_dockingapproachpoint.h"
#include "bt_docking.h"
#include "bt_drivebackwards.h"
#include "bt_status.h"

#define SERVICE_CLIENT_WAIT_TIMEOUT     60.0        /* amount of time to wait for services to become available */
#define ACTION_CLIENT_WAIT_TIMEOUT    1800.0        /* amount of time to wait for action servers, if there is no GPS, MBF will not come up, so we need a substanital amount of time potentially */

using namespace BT;

/*******************************************/
/* global states used by nodes passed by & */
/*******************************************/
/* origin /mowgli/status */
bool gstate_blade_motor_enabled;
bool gstate_is_charging;    // TODO: we need a proper gstate_is_docked ! - is_charging might be flapping when the charge PWM adjusts even if in dock
float gstate_v_battery;
float gstate_v_charge;
uint8_t gstate_highlevel_command = 0;

ros::Time gstate_status_last_time(0.0);
/* origin /odom */
nav_msgs::Odometry gstate_odom;
ros::Time gstate_odom_last_time(0.0);

/***************/
/* Subscribers */
/***************/
ros::Subscriber subMowgliStatus;            /* /mowgli/status */
ros::Subscriber subOdom;                    /* /odom */

/**************/
/* Publishers */
/**************/
ros::Publisher pubCurrentState;             /* /mower_logic/current_state */
ros::Publisher pubCommandVelocity;          /* /logic_vel */

/*******************/
/* Service clients */
/*******************/
ros::ServiceClient srvGpsControlClient;     /* /mower_service/set_gps_state */
ros::ServiceClient srvMowClient;            /* /mower_service/mow_enabled */
ros::ServiceClient srvDockingPointClient;   /* /mower_map_service/get_docking_point */

/*******************/
/* Service servers */
/*******************/
ros::ServiceServer srvHighLevelCommand;     /* /mower_service/high_level_control */

/*************************/
/* Simple Action clients */
/*************************/
actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *srvMbfExePathClient;    /* /move_base_flex/exe_path */
actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *srvMbfMoveBaseClient;  /* /move_base_flex/move_base */

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

/// @brief Halt all bot movement
void stopMoving() 
{
    ROS_INFO_STREAM("mowgli_bt: stopMoving() - stopping bot movement");

    geometry_msgs::Twist stop;
    stop.angular.z = 0;
    stop.linear.x = 0;
    pubCommandVelocity.publish(stop);
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


/// @brief wait for all action servers required to become available
/// @param  
/// @return true if all are found, false if one is not found within a given timeout
bool waitForServers(void)
{
    ROS_INFO("mowgli_bt: Waiting for MoveBaseFlex/ExecPathClient server ...");
    if (!srvMbfExePathClient->waitForServer(ros::Duration(ACTION_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for MoveBaseFlex/ExecPathClient server");        
        return false;
    }
    ROS_INFO("mowgli_bt: Waiting for MoveBaseFlex/MoveBaseClient server ...");
    if (!srvMbfMoveBaseClient->waitForServer(ros::Duration(ACTION_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for MoveBaseFlex/MoveBaseClient server");        
        return false;
    }
    return true; // all action servers present
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
    ROS_INFO("mowgli_bt: Waiting for DockingPoint service ...");
    if (!srvDockingPointClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for DockingPoint service");        
        return false;
    }    
    return true; // all services present
}


/// @brief /mowgli/status subscriber callback
/// @param msg status message
void MowgliStatusCB(const mowgli::status::ConstPtr &msg) 
{    
#ifdef BT_DEBUG    
    ROS_INFO_STREAM("mowgli_bt: MowgliStatusCB");
#endif
    gstate_blade_motor_enabled = msg->blade_motor_enabled;
    gstate_v_battery = msg->v_battery;
    gstate_v_charge = msg->v_charge;
    gstate_is_charging = msg->is_charging;    
    gstate_status_last_time = ros::Time::now();
}

/// @brief /odom subscriber callback
/// @param msg status message
void OdomCB(const nav_msgs::Odometry::ConstPtr &msg) 
{    
#ifdef BT_DEBUG
    ROS_INFO_STREAM("mowgli_bt: OdomCB");
#endif
    gstate_odom = *msg;    
    gstate_odom_last_time = ros::Time::now();
}


/// @brief callback for the /mower_service/high_level_control service
/// @param req HighLevelControlSrv
/// @param res HighLevelControlSrv
/// @return always true
bool highLevelCommandCB(mower_msgs::HighLevelControlSrvRequest &req, mower_msgs::HighLevelControlSrvResponse &res) 
{
    gstate_highlevel_command = req.command;
    switch(req.command) {
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME: 
	        ROS_INFO_STREAM("mowgli_bt: received COMMAND_HOME");
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_START:
		    ROS_INFO_STREAM("mowgli_bt: received COMMAND_START");
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_S1:
		    ROS_INFO_STREAM("mowgli_bt: received COMMAND_S1");
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_S2:
	        ROS_INFO_STREAM("mowgli_bt: received COMMAND_S2"); 
            break;
    }
    return true;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mowgli_bt");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    signal(SIGINT, mySigintHandler);
    ROS_INFO_STREAM("mowgli_bt: Starting mowgli_bt");

    ros::AsyncSpinner asyncSpinner(2);  
    asyncSpinner.start();

    // Subscribers
    subMowgliStatus = n.subscribe("mowgli/status", 50, MowgliStatusCB);
    subOdom = n.subscribe("odom", 50, OdomCB);

    // Publishers
    pubCurrentState = n.advertise<std_msgs::String>("mower_logic/current_state", 10, true);
    pubCommandVelocity = n.advertise<geometry_msgs::Twist>("/logic_vel", 1, true);

    // Service servers
    ros::ServiceServer srvHighLevelCommand = n.advertiseService("mower_service/high_level_control", highLevelCommandCB);

    // Service clients
    srvGpsControlClient = n.serviceClient<mower_msgs::GPSControlSrv>("mower_service/set_gps_state");
    srvMowClient =  n.serviceClient<mower_msgs::MowerControlSrv>("mower_service/mow_enabled");
    srvDockingPointClient = n.serviceClient<mower_map::GetDockingPointSrv>("mower_map_service/get_docking_point");

    // Action clients
    srvMbfExePathClient = new actionlib::SimpleActionClient<mbf_msgs::ExePathAction>("move_base_flex/exe_path", true);
    srvMbfMoveBaseClient = new actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>("move_base_flex/move_base", true);

    // Wait for servers
    if (!waitForServers())
    {
        ros::shutdown(); // bye bye
        exit(-1);
    }
    ROS_INFO_STREAM("mowgli_bt: All action servers are now available");

    // Wait for services
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

    // bt_dockingapproachpoint (DockingApproachPoint)
    NodeBuilder builder_DockingApproachPoint =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<DockingApproachPoint>( name, config, srvMbfMoveBaseClient, srvDockingPointClient);
    };
    factory.registerBuilder<DockingApproachPoint>( "DockingApproachPoint", builder_DockingApproachPoint);


    // bt_dockingapproach (DockingApproach)
    NodeBuilder builder_DockingApproach =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<DockingApproach>( name, config, srvMbfExePathClient, srvDockingPointClient);
    };
    factory.registerBuilder<DockingApproach>( "DockingApproach", builder_DockingApproach);

    // bt_dockingapproach (Docking)
    NodeBuilder builder_Docking =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<Docking>( name, config, srvMbfExePathClient, srvDockingPointClient, &gstate_v_charge);     // TODO: introduce an "is_docked" status variable
    };
    factory.registerBuilder<Docking>( "Docking", builder_Docking);


    // bt_drivebackwards (DriveBackwards)
    NodeBuilder builder_DriveBackwards =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<DriveBackwards>( name, config, srvMbfExePathClient, &gstate_odom);
    };
    factory.registerBuilder<DriveBackwards>( "DriveBackwards", builder_DriveBackwards);


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

    // bt_status (IsCharging)
    NodeBuilder builder_IsCharging =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<IsCharging>( name, config, &gstate_is_charging );
    };
    factory.registerBuilder<IsCharging>( "IsCharging", builder_IsCharging);

    // bt_status (IsMowing)
    NodeBuilder builder_IsMowing =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<IsMowing>( name, config, &gstate_blade_motor_enabled );
    };
    factory.registerBuilder<IsMowing>( "IsMowing", builder_IsMowing);

    // bt_status (GetHighLevelCommand)
    
    NodeBuilder builder_GetHighLevelCommand =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetHighLevelCommand>( name, config, &gstate_highlevel_command );
    };

    factory.registerBuilder<GetHighLevelCommand>( "GetHighLevelCommand", builder_GetHighLevelCommand);

    // Load tree from XML
    auto tree = factory.createTreeFromFile("../MowgliRover/src/mowgli/src/tree.xml");
    PublisherZMQ publisher_zmq(tree); // for Groot

    ros::Rate r(1); // ROS loop rate
    while (ros::ok()) {
       //  ROS_INFO_STREAM("gstate_blade_motor_enabled: " << gstate_blade_motor_enabled);
        r.sleep();        
        tree.tickRoot();
    }
    return 0;
}
