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


// state machine to enable/disable blade in mowgli because we need to send keepalives
// or mowgli will stop the blade after 25secs - service calls are heavy on rosserial - so we only send when needed
bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
//    ROS_INFO("mowgli_proxy: setMowEnabled = %d/%d", req.mow_enabled, mowEnabledFlag);
    // DISABLED -> ENABLED
    if (req.mow_enabled && !mowEnabledFlag)
    {
        ROS_WARN_STREAM("mowgli_blade: Blade DISABLED -> ENABLED");
        sendMowEnabled(true);
    }        
    // ENABLED -> DISABLED
    else if (!req.mow_enabled && mowEnabledFlag)
    {
        ROS_WARN_STREAM("mowgli_blade: Blade ENABLED -> DISABLED");
        sendMowEnabled(false);
    }
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
    // the blade motor times out after 25sec
    ros::Duration duration(20.0);
    while (ros::ok()) {
        if (mowEnabledFlag)
        {
            sendMowEnabled(mowEnabledFlag);
        }
        duration.sleep();
        ros::spinOnce(); 
    }
    return 0;
}
