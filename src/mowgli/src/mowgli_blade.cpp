/*
 * Mowgli Blade Manager V 1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 *
 * v1.0: inital release
 *
 */

#include <signal.h>
#include "ros/ros.h"

/* OM */
#include <mower_msgs/MowerControlSrv.h>
#include <mower_msgs/EmergencyStopSrv.h>


/* Mowgli */
#include "std_srvs/SetBool.h"

// Service Clients
ros::ServiceClient mowClient;
bool mowEnabledFlag = false;
ros::Time last_mowEnabledFlagSent(0.0);

// tell mowgli to enable/disable mower
void sendMowEnabled(bool ena_dis)
{
    std_srvs::SetBool mow_srv;
    mow_srv.request.data = ena_dis;
    mowEnabledFlag = ena_dis;
   // ROS_INFO("-------------------------------------");
   // ROS_INFO("mowgli_blade: sendMowEnabled = %d ", ena_dis);
    mowClient.call(mow_srv);
   // ROS_INFO("-------------------------------------");    
}

// make sure we stop the blade on exit
void mySigintHandler(int sig)
{
    sendMowEnabled(false); 
    ros::shutdown();
}


/// @brief state machine to enable/disable blade in mowgli because we need to send keepalives
/// @param req service request data
/// @param res service response data
/// @return always true
bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
    ros::Time started = ros::Time::now();
    ROS_WARN("#### mowgli_blade: setMowEnabled call = (%d to %d)", mowEnabledFlag, req.mow_enabled);
    // DISABLED -> ENABLED
    if (req.mow_enabled && !mowEnabledFlag)
    {
        ROS_WARN_STREAM("#### mowgli_blade: Blade DISABLED -> ENABLED");
        sendMowEnabled(true);
    }        
    // ENABLED -> DISABLED
    else if (!req.mow_enabled && mowEnabledFlag)
    {
        ROS_WARN_STREAM("#### mowgli_blade: Blade ENABLED -> DISABLED");
        sendMowEnabled(false);
    }
    ROS_WARN_STREAM("#### mowgli_blade: setMowEnabled call completed" << mowEnabledFlag << ") call completed within " << (ros::Time::now()-started).toSec() << "s");
    return true;
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mowgli_blade");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ROS_INFO_STREAM("mowgli_blade: Starting mowgli_blade manager");

    signal(SIGINT, mySigintHandler);

    // Services required by OpenMower
    ros::ServiceServer om_mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);


    // Mowgli Services
    mowClient = n.serviceClient<std_srvs::SetBool>("mowgli/EnableMowerMotor");

    ROS_INFO("mowgli_blade: Waiting for mowgli/EnableMowerMotor server");
    if (!mowClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("mowgli_blade: EnableMowerMotor server service not found.");
        return 1;
    }

    // blocking loop to send out sendMowEnabled() messages to mowgli as
    // the blade motor times out after 25sec we refresh the state every 20sec 
    ros::Time last_keepalive_time = ros::Time::now();
    ros::Rate r(0.5); // ROS loop rate
    while (ros::ok()) {
        if (mowEnabledFlag && (ros::Time::now()-last_keepalive_time).toSec() > 20.0) // if BLADE motor should be on we send out keepalives every 20s
        {
            ROS_WARN("mowgli_blade: sendMowEnabled(%d) 20sec keepalive", mowEnabledFlag);
            sendMowEnabled(mowEnabledFlag);
            last_keepalive_time = ros::Time::now();
        }
        r.sleep();
        ros::spinOnce(); 
    }
    return 0;
}
