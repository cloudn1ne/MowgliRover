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
#include "loggers/bt_statelogger.h"                             // custom logger that ends out state transitions via a ROS Topic
//#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"         // File logger
// #include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"    // JSON logger


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
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "ftc_local_planner/PlannerGetProgress.h"
#include "mowgli/status.h"



#include "bt_nodes.h"
#include "bt_gpscontrol.h"
#include "bt_blademotorcontrol.h"
#include "bt_dockingapproach.h"
#include "bt_dockingapproachpoint.h"
#include "bt_docking.h"
#include "bt_drivebackward.h"
#include "bt_driveforward.h"
#include "bt_status.h"
#include "bt_getmowplan.h"
#include "bt_getmowpath.h"
#include "bt_getfirstpose.h"
#include "bt_approachpose.h"
#include "bt_followpath.h"
#include "bt_trimposes.h"
#include "bt_env.h"
#include "bt_generate2dpose.h"


// BT primitives
#include "bt_forloop.h"

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
bool gstate_odom_valid = false;
ros::Time gstate_odom_last_time(0.0);
uint32_t valid_odom_counter;

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
ros::ServiceClient svcGpsControlClient;         /* /mower_service/set_gps_state */
ros::ServiceClient svcMowClient;                /* /mower_service/mow_enabled */
ros::ServiceClient svcDockingPointClient;       /* /mower_map_service/get_docking_point */
ros::ServiceClient svcMapClient;                /* /mower_map_service/get_mowing_area */
ros::ServiceClient svcPlanPathClient;           /* /slic3r_coverage_planner/plan_path */
ros::ServiceClient svcPlannerGetProgressClient; /* /move_base_flex/FTCPlanner/planner_get_progress */
ros::ServiceClient svcPathProgressClient;       

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
    svcGpsControlClient.call(srv);
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
    if (!svcGpsControlClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for GpsControl service");        
        return false;
    }
    ROS_INFO("mowgli_bt: Waiting for Mow service ...");
    if (!svcMowClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for Mow service");        
        return false;
    }
    ROS_INFO("mowgli_bt: Waiting for DockingPoint service ...");
    if (!svcDockingPointClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for DockingPoint service");        
        return false;
    }    
    ROS_INFO("mowgli_bt: Waiting for Map service ...");
    if (!svcMapClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for Map service");        
        return false;
    }    
    ROS_INFO("mowgli_bt: Waiting for PlanPath service ...");
    if (!svcPlanPathClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for PlanPath service");        
        return false;
    }      
    ROS_INFO("mowgli_bt: Waiting for PlannerGetProgress service ...");
    if (!svcPlannerGetProgressClient.waitForExistence(ros::Duration(SERVICE_CLIENT_WAIT_TIMEOUT, 0.0))) {
        ROS_ERROR("mowgli_bt: Timeout while waiting for PlannerGetProgress service");        
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

/// @brief Called every 0.5s, update gstate_odom_valid
/// @param timer_event 
void checkOdom(const ros::TimerEvent &timer_event) {
    if ( (ros::Time::now() - gstate_odom_last_time).toSec() < 1.0 ) // /odom received within last second
    {
            valid_odom_counter++;
            if (valid_odom_counter > 5 )
            {
                gstate_odom_valid = true;
            }
    }
    else
    {
            valid_odom_counter = 0; // reset
            gstate_odom_valid = false;
    }
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
    svcGpsControlClient = n.serviceClient<mower_msgs::GPSControlSrv>("mower_service/set_gps_state");
    svcMowClient =  n.serviceClient<mower_msgs::MowerControlSrv>("mower_service/mow_enabled");
    svcDockingPointClient = n.serviceClient<mower_map::GetDockingPointSrv>("mower_map_service/get_docking_point");
    svcMapClient = n.serviceClient<mower_map::GetMowingAreaSrv>("mower_map_service/get_mowing_area");
    svcPlanPathClient = n.serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");
    svcPlannerGetProgressClient = n.serviceClient<ftc_local_planner::PlannerGetProgress>("move_base_flex/FTCPlanner/planner_get_progress");

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

    // start /odom state engine
    ros::Timer odomCheckTimer = n.createTimer(ros::Duration(0.5), checkOdom);

    BehaviorTreeFactory factory;   

    // bt_nodes 
    using namespace DummyNodes;    
    factory.registerNodeType<SaySomething>("SayString");
    factory.registerNodeType<SayFloat>("SayFloat");
    factory.registerNodeType<SayInt>("SayInt");
    factory.registerNodeType<SayBool>("SayBool");
    factory.registerNodeType<SleepNode>("SleepNode");
    

    // Mowgli Nodes
    // bt_gpscontrol
    factory.registerNodeType<GpsControl>("GpsControl");
    // bt_forloop
    factory.registerNodeType<ForLoop>("ForLoop");
    // bt_getmowpath
    factory.registerNodeType<GetMowPath>("GetMowPath");
    // bt_getfirstpose
    factory.registerNodeType<GetFirstPose>("GetFirstPose");
    // bt_trimposes
    factory.registerNodeType<TrimPoses>("TrimPoses");
    // bt_generate2dpose
    factory.registerNodeType<Generate2DPose>("Generate2DPose");

    // bt_followpath (FollowPath)
    NodeBuilder builder_FollowPath =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<FollowPath>( name, config, srvMbfExePathClient, svcPlannerGetProgressClient);
    };
    factory.registerBuilder<FollowPath>( "FollowPath", builder_FollowPath);

    // bt_approachpose (ApproachPose)
    NodeBuilder builder_ApproachPose =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<ApproachPose>( name, config, srvMbfMoveBaseClient);
    };
    factory.registerBuilder<ApproachPose>( "ApproachPose", builder_ApproachPose);

    // bt_getmowplan (GetMowPlan)    
    NodeBuilder builder_GetMowPlan =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetMowPlan>( name, config,  svcMapClient, svcPlanPathClient);
    };
    factory.registerBuilder<GetMowPlan>( "GetMowPlan", builder_GetMowPlan);

    // bt_dockingapproachpoint (DockingApproachPoint)
    NodeBuilder builder_DockingApproachPoint =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<DockingApproachPoint>( name, config, srvMbfMoveBaseClient, svcDockingPointClient);
    };
    factory.registerBuilder<DockingApproachPoint>( "DockingApproachPoint", builder_DockingApproachPoint);


    // bt_dockingapproach (DockingApproach)
    NodeBuilder builder_DockingApproach =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<DockingApproach>( name, config, srvMbfExePathClient, svcDockingPointClient);
    };
    factory.registerBuilder<DockingApproach>( "DockingApproach", builder_DockingApproach);

    // bt_dockingapproach (Docking)
    NodeBuilder builder_Docking =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<Docking>( name, config, srvMbfExePathClient, svcDockingPointClient, &gstate_v_charge);     // TODO: introduce an "is_docked" status variable
    };
    factory.registerBuilder<Docking>( "Docking", builder_Docking);


    // bt_drivebackward (DriveBackward)
    NodeBuilder builder_DriveBackward =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<DriveBackward>( name, config, srvMbfExePathClient, &gstate_odom);
    };
    factory.registerBuilder<DriveBackward>( "DriveBackward", builder_DriveBackward);

    // bt_driveforward (DriveForward)
    NodeBuilder builder_DriveForward =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<DriveForward>( name, config, srvMbfExePathClient, &gstate_odom);
    };
    factory.registerBuilder<DriveForward>( "DriveForward", builder_DriveForward);

    // bt_blademotorcontrol (BladeMotorControl)
    NodeBuilder builder_BladeMotorControl =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<BladeMotorControl>( name, config, svcMowClient, &gstate_blade_motor_enabled );
    };
    factory.registerBuilder<BladeMotorControl>( "BladeMotorControl", builder_BladeMotorControl);

    // bt_status (GetMowerBatteryVoltage)
    NodeBuilder builder_GetMowerBatteryVoltage =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetMowerBatteryVoltage>( name, config, &gstate_v_battery );
    };
    factory.registerBuilder<GetMowerBatteryVoltage>( "GetMowerBatteryVoltage", builder_GetMowerBatteryVoltage);

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

    // bt_status (IsOdomValid)
    NodeBuilder builder_IsOdomValid =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<IsOdomValid>( name, config, &gstate_odom_valid );
    };
    factory.registerBuilder<IsOdomValid>( "IsOdomValid", builder_IsOdomValid);

    // bt_status (WaitForOdom)
    NodeBuilder builder_WaitForOdom =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<WaitForOdom>( name, config, &gstate_odom_valid );
    };
    factory.registerBuilder<WaitForOdom>( "WaitForOdom", builder_WaitForOdom);

    // bt_status (GetHighLevelCommand)    
    NodeBuilder builder_GetHighLevelCommand =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetHighLevelCommand>( name, config, &gstate_highlevel_command );
    };
    factory.registerBuilder<GetHighLevelCommand>( "GetHighLevelCommand", builder_GetHighLevelCommand);

    // bt_env (GetEnvString)    
    NodeBuilder builder_GetEnvString =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetEnvString>( name, config );
    };
    factory.registerBuilder<GetEnvString>( "GetEnvString", builder_GetEnvString);

    // bt_env (GetEnvInt)    
    NodeBuilder builder_GetEnvInt =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetEnvInt>( name, config );
    };
    factory.registerBuilder<GetEnvInt>( "GetEnvInt", builder_GetEnvInt);

    // bt_env (GetEnvFloat)    
    NodeBuilder builder_GetEnvFloat =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetEnvFloat>( name, config );
    };
    factory.registerBuilder<GetEnvFloat>( "GetEnvFloat", builder_GetEnvFloat);

    // bt_env (GetEnvBool)    
    NodeBuilder builder_GetEnvBool =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetEnvBool>( name, config );
    };
    factory.registerBuilder<GetEnvBool>( "GetEnvBool", builder_GetEnvBool);

    // bt_env (GetEnvBoolAsInt)    
    NodeBuilder builder_GetEnvBoolAsInt =
    [](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<GetEnvBoolAsInt>( name, config );
    };
    factory.registerBuilder<GetEnvBoolAsInt>( "GetEnvBoolAsInt", builder_GetEnvBoolAsInt);

    // Load tree from XML
    std::string xml_path;    
    if(!paramNh.getParam("xml_path", xml_path)) {
        ROS_ERROR_STREAM("mowgli_bt: xml_path parameter missing.");
        return 1;
    }

    ROS_INFO_STREAM("mowgli_bt: Loading " << xml_path);
    auto tree = factory.createTreeFromFile(xml_path);
    PublisherZMQ publisher_zmq(tree); // for Groot

    StateLogger state_logger(tree, pubCurrentState);

    // This logger stores the execution time of each node
    // MinitraceLogger logger_minitrace(tree, "/tmp/bt_trace.json");

    // File Logger
    //FileLogger logger_file(tree, "/tmp/bt_trace.fbl");

    ros::Rate r(1); // ROS loop rate
    while (ros::ok()) {       
        r.sleep();        
        tree.tickRoot();
    }
    return 0;
}
