/*
 * Mowgli to OpenMower Proxy V 1.1
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 *
 * v1.0: inital release
 * v1.1: DR experimentaly code added
 *
 */

#include "ros/ros.h"

/* OM */
#include <mower_msgs/Status.h>
#include <mower_msgs/ESCStatus.h>
// #include <mower_msgs/MowerControlSrv.h>
#include <mower_msgs/GPSControlSrv.h>
#include <mower_msgs/EmergencyStopSrv.h>

/* Mowgli */
#include "std_srvs/SetBool.h"
#include <mowgli/MowgliProxyConfig.h>
#include <mowgli/status.h>


/* GPS/ODOM */
#include <nav_msgs/Odometry.h>
#include <ublox_msgs/NavPVT.h>
// #include <ublox_msgs/NavRELPOSNED9.h>

/* TF */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <robot_localization/navsat_conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

// #define MOWGLI_DEBUG  1
#define MOWGLI_DR_DEBUG 1

// Dynamic Reconfigure for MowgliProxy
mowgli::MowgliProxyConfig config;

// OM Status
// mower_msgs::Status om_mower_status;

// GPS
double datumN, datumE, datumLat, datumLng;  // GPS datums in N/E and LAT/LONG formats
std::string datumZone;
tf2::Vector3 last_gps_pos;
double last_gps_acc_m;
ros::Time last_gps_odometry_time(0.0);
int gps_quality_flags = 0;
int gps_outlier_count = 0;
int valid_gps_samples = 0;
bool gpsRTKOdometryValid = false;
bool gpsOdometryIsRTK = false;
bool gpsEnabled = true;


// IMU
//sensor_msgs::Imu lastImu;
bool hasImuMessage = false;
geometry_msgs::Quaternion mowgli_orientation;


// ODOM
double x = 0, y = 0, r = 0;
double vx = 0, vy = 0, vr = 0;
double delta_x = 0, delta_y = 0;

// Mowgli ODOM (Dead Reckoning)
// double dr_x_offset = 0, dr_y_offset = 0;        // offset (between RTK and Mowgli Odom) is updated with every valid RTK fix
double dr_x = 0, dr_y = 0;                         // coordinates that are then tracked by DR alone
double dr_x_old = 0, dr_y_old = 0;
double dr_x_offset = 0, dr_y_offset = 0;           // DR offsets between GPS-RKT and /mowgli/odom - synced whenever we have a GPS-RTK fix

bool dr_valid_offsets = false;
bool dr_active = false;
bool dr_possible = false;
bool dr_update_offsets = false;

ros::Time dr_starttime(0.0);



// DR ticks based
unsigned long dr_left_encoder_ticks = 0;
unsigned long dr_right_encoder_ticks = 0;

unsigned long dr_left_encoder_ticks_offset = 0;
unsigned long dr_right_encoder_ticks_offset = 0;

// /set_pose in EKF as long as we have valid RTK-GPS fixes
geometry_msgs::PoseWithCovarianceStamped dr_lastpose;
ros::Time dr_lastpost_time(0.0);

// Pubs
ros::Publisher pubOMStatus;
ros::Publisher pubOdom;
ros::Publisher pubSetPose;

// Subs
ros::Subscriber subGPSPosFix;           // /ublox/fix (GPS Coords)
ros::Subscriber subGPSQuality;          // /ublox/navpvt (GPS FIX Quality)
ros::Subscriber subIMUData;
ros::Subscriber subMowgliStatus;
ros::Subscriber subMowgliOdom;
ros::Subscriber subEKFOdom;

// Service Clients
ros::ServiceClient mowClient;
bool mowEmergencyDisableFlag = false;          

// Debug 
bool debug_gps_invalid = false;


// TF
//tf2_ros::Buffer tfBuffer; 
//tf::TransformBroadcaster transform_broadcaster;
//tf2_ros::TransformListener tfListener(tfBuffer);
//geometry_msgs::TransformStamped transformStamped;

bool setDebugGPSInvalid(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res)
{
    ROS_INFO_STREAM("mowgli_proxy: setDebugGPSInvalid(" << req.data << ")");
    debug_gps_invalid = req.data;
    return true;
}

bool setGpsState(mower_msgs::GPSControlSrvRequest &req, mower_msgs::GPSControlSrvResponse &res) {    
    ROS_INFO_STREAM("mowgli_proxy: setGpsState =" << gpsEnabled);
    gpsEnabled = req.gps_enabled;
    gpsRTKOdometryValid = false;    
    return true;
}



bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
    ROS_ERROR_STREAM("mowgli_proxy: setEmergencyStop");
    if (!mowEmergencyDisableFlag)
    {
        mowEmergencyDisableFlag = true;
        std_srvs::SetBool mow_srv;
        mow_srv.request.data = false;
        mowClient.call(mow_srv);
        ROS_WARN_STREAM("mowgli_proxy: EMERGENCY Blade ENABLED -> DISABLED");
    }
    return true;
}

/*
 * Publish Odometry and base_link->map transform
 *
 * Data is either RTK-GPS data if available
 * or EKF Dead Reckoning data 
 */ 
void pubOdometry(double odom_x, double odom_y, double odom_vx, double odom_vr, geometry_msgs::Quaternion orientation) {

    // ROS_INFO_STREAM("mowgli_proxy: pubOdometry");
    static tf2_ros::TransformBroadcaster transform_broadcaster;
    
    ros::Time current_time = ros::Time::now();

    /* Transform */
    geometry_msgs::TransformStamped odom_trans;    
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = odom_x;
    odom_trans.transform.translation.y = odom_y;
    // TODO: Add logic for 3d odometry
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = orientation;
    transform_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    // set the position
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_trans.transform.rotation;
    // set the velocity    
    odom_msg.twist.twist.linear.x = odom_vx;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = odom_vr;

    odom_msg.pose.covariance = {
            10000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10000.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.00001
    };

  //  ROS_INFO_STREAM(" gpsOdometryValid = " << gpsOdometryValid);
  //  ROS_INFO_STREAM(" gpsEnabled = " << gpsEnabled);

    if (gpsRTKOdometryValid && gpsEnabled && (ros::Time::now() - last_gps_odometry_time) < ros::Duration(5.0)) {
        odom_msg.pose.covariance[0] = last_gps_acc_m * last_gps_acc_m;
        odom_msg.pose.covariance[7] = last_gps_acc_m * last_gps_acc_m;
    }
   
    odom_msg.twist.covariance = {
            0.000001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.000001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.000001
    };

 //   ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: pubOdometry(/odom) [ GPS-RTK?: %s, DR?: %s] x = %0.2fm, y = %0.2fm, yaw = %0.2fdeg", (gpsOdometryIsRTK?"yes":"no"), (dr_active?"yes":"no"), x, y, r*180/M_PI);
 //   if (dr_valid_offsets)
 //   {
 //       ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: pubOdometry(/odom) [ DR Method 2 Coords   ] x = %0.2fm, y = %0.2fm",  dr_x+dr_x_offset, dr_y+dr_y_offset);
 //   }

    if (dr_active)
        ROS_INFO_DELAYED_THROTTLE(1, "mowgli_proxy: pubOdometry(/odom) [ DR ] x = %0.2fm, y = %0.2fm, yaw = %0.2fdeg", odom_x, odom_y, r*180/M_PI);
    else
    {
        unsigned int carrSoln = (gps_quality_flags >> 6) & 3;
        ROS_INFO_DELAYED_THROTTLE(1, "mowgli_proxy: pubOdometry(/odom) [ GPS %s] x = %0.2fm, y = %0.2fm, yaw = %0.2fdeg", (carrSoln==0x2)?"RTK-Fixed":"RTK-Float", odom_x, odom_y, r*180/M_PI);
    }
    // publish the message
    pubOdom.publish(odom_msg);
}

/*
 * Update the pose of the DR EKF filter
 * (called when we have a proper GPS-RTK fix)
 */
void updateDeadReckoning(double gps_base_link_x, double gps_base_link_y)
{
    // update EKF every <dr_update_interval> seconds
    if ((ros::Time::now()-dr_lastpost_time).toSec() > config.dr_update_interval)
    {
        // ROS_WARN("mowgli_proxy: DR Pose synced with EKF");
        dr_lastpose.pose.pose.position.x = gps_base_link_x;    
        dr_lastpose.pose.pose.position.y = gps_base_link_y;
        dr_lastpose.pose.pose.position.z = 0; // we cant fly (yet)    
        dr_lastpose.pose.pose.orientation = mowgli_orientation; 
        dr_lastpose.header.stamp = ros::Time::now();
        dr_lastpose.header.frame_id = "map";
        pubSetPose.publish(dr_lastpose);
        dr_lastpost_time = ros::Time::now();
        dr_possible = true;
    }
}


/*
 * Perform dead reckoning based on /mowgli/odom 
 */
void doDeadReckoning()
{
    if (!dr_possible)
    {
        ROS_WARN_STREAM_THROTTLE(1, "mowgli_proxy: Dead Reckoning requested but we didnt have a recent GPS RTK fix - EKF is not up to date !");
    }
    else
    {    
        if (!dr_active)
        {
            ROS_WARN("mowgli_proxy: doDeadReckoning: activated");
            dr_starttime = ros::Time::now();
            dr_active = true;
        }
        if (dr_active)
        {
            if ((ros::Time::now()-dr_starttime).toSec() < config.dr_max_duration_sec)            
            {
                dr_active = true; // this flag will cause EKFOdomCB to emit the DR Odom data as /odom
            }
            else
            {
                ROS_ERROR("mowgli_proxy: doDeadReckoning: Maximum Dead Reckoning (%d sec) time reached", config.dr_max_duration_sec);
                dr_possible = false; // block another DR attempt until we get a updateDeadReckoning()
                dr_active = false;
            }
        }        
    }
}

void handleGPSUpdate(tf2::Vector3 gps_pos, double gps_accuracy_m) {   
    double time_since_last_gps = (ros::Time::now() - last_gps_odometry_time).toSec();

    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("mowgli_proxy: last GPS position was received " << time_since_last_gps << " seconds ago.");
        gpsRTKOdometryValid = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps_pos = gps_pos;
        last_gps_acc_m = gps_accuracy_m;
        last_gps_odometry_time = ros::Time::now();

        doDeadReckoning();
        return;
    }
    

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();
#ifdef MOWGLI_DR_DEBUG    
   // ROS_INFO_STREAM("mowgli_proxy: distance_to_last_gps = " << distance_to_last_gps );
#endif   
    if (distance_to_last_gps <= config.max_distance_to_last_gps_pos) 
    {
        // inlier, we treat it normally
        // calculate current base_link position from orientation and distance parameter
        double gps_base_link_x = gps_pos.x() - config.gps_antenna_offset * cos(r);
        double gps_base_link_y = gps_pos.y() - config.gps_antenna_offset * sin(r);
        //ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: gps_pos.x/y   x = %0.2f y = %0.2f", gps_pos.x(), gps_pos.y());
        //ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: base_link_x/y x = %0.2f y = %0.2f", base_link_x, base_link_y);

        last_gps_pos = gps_pos;
        last_gps_acc_m = gps_accuracy_m;
        last_gps_odometry_time = ros::Time::now();
        gps_outlier_count = 0;
        valid_gps_samples++;

        if (!gpsRTKOdometryValid && gpsOdometryIsRTK && valid_gps_samples > 10) {
            ROS_WARN_STREAM("mowgli_proxy: received 10 valid RTK GPS fixes, gpsOdometry is now considered valid");
            ROS_WARN_STREAM("mowgli_proxy: updated base_link coords are: " << gps_base_link_x << ", " << gps_base_link_y);
            gpsRTKOdometryValid = true;
        }
        
        if (gpsRTKOdometryValid)
        {
            x = gps_base_link_x;
            y = gps_base_link_y;       
            updateDeadReckoning(gps_base_link_x, gps_base_link_y);                
        }
        else
        {
            x = x * (1.0 - config.gps_filter_factor) + config.gps_filter_factor * gps_base_link_x;
            y = y * (1.0 - config.gps_filter_factor) + config.gps_filter_factor * gps_base_link_y;
        }       
    } 
    else 
    {
        ROS_WARN_STREAM("mowgli_proxy: New GPS data is more than " << config.max_distance_to_last_gps_pos << " away from last fix. The calculated distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) 
        {
            ROS_ERROR_STREAM("mowgli_proxy: Too many GPS outliers, we are now assuming that the current gps values are invalid.");
            last_gps_pos = gps_pos;
            last_gps_acc_m = gps_accuracy_m;
            last_gps_odometry_time = ros::Time::now();
            gpsRTKOdometryValid = false;
            valid_gps_samples = 0;
            gps_outlier_count = 0;

            doDeadReckoning();
            return;
        }
    }
    dr_active = false;
    pubOdometry(x,y, vx, vr, mowgli_orientation);
}

void EKFOdomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO_STREAM("mowgli_proxy: EKFOdomCB");
    if (dr_active)
    {
        // TODO: vr is taken from EKF, but send the mowgli_orientation 
        // we should extract the right vr from mowgli_orientation

        //ROS_INFO_STREAM("mowgli_proxy: EKFOdomCB override active");
        pubOdometry(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->twist.twist.linear.x, msg->twist.twist.angular.z, mowgli_orientation);   // pipe EKF /odometry/filtered to -> /odom as is
    }
}

void MowgliOdomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
#ifdef MOWGLI_DEBUG    
    ROS_INFO_STREAM("mowgli_proxy: MowgliOdomCB");
#endif
/*
    dr_x = msg->pose.pose.position.x;
    dr_y = msg->pose.pose.position.y;
    vx = msg->twist.twist.linear.x;
    vy = 0;
    vr = msg->twist.twist.angular.z;
    */
    // deltas
    /*
    delta_x = dr_x_old - dr_x;
    delta_y = dr_y_old - dr_y;    
    dr_x_old = dr_x;
    dr_y_old = dr_y;
*/
    // update pose for /set_pose in EKF
    //dr_lastpose.pose = msg->pose;       // x,y
    //dr_lastpose.pose.pose.orientation = mowgli_orientation; // y
    //dr_lastpose.header = msg->header;
  //  ROS_INFO_DELAYED_THROTTLE(1, "mowgli_proxy: MowgliOdomCB x = %0.2f y = %0.2f", x, y);
  //  ROS_INFO("mowgli_proxy: MowgliOdomCB delta_x = %0.2f delta_y = %0.2f", delta_x, delta_y);
}

/*
* translate /mowgli/status to /mower/status
*/
void MowgliStatusCB(const mowgli::status::ConstPtr &msg) 
{
    mower_msgs::Status om_mower_status;
#ifdef MOWGLI_DEBUG    
    ROS_INFO_STREAM("mowgli_proxy: MowgliStatusCB");
#endif    

    om_mower_status.v_battery = msg->v_battery;
    om_mower_status.v_charge = msg->v_charge;
    om_mower_status.charge_current = msg->i_charge;
    om_mower_status.emergency = msg->emergency_stopbutton_triggered | msg->emergency_tilt_mech_triggered | msg->emergency_tilt_accel_triggered |
                                msg->emergency_right_wheel_lifted | msg->emergency_left_wheel_lifted;
    om_mower_status.rain_detected = msg->rain_detected;

    // the following attributes are not present in Mowgli, so we fake em
    om_mower_status.left_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    om_mower_status.right_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    om_mower_status.mow_esc_status.temperature_motor = 20;

//    dr_left_encoder_ticks = msg->left_encoder_ticks;
//    dr_right_encoder_ticks = msg->right_encoder_ticks;

    pubOMStatus.publish(om_mower_status);
}


/*
 * GPS quality extracted from /ublox/navpvt (flags)
 * We want to know if we have a solid 3D FIX
 */
void GPSFixQualityCB(const ublox_msgs::NavPVT::ConstPtr &msg) 
{        
#ifdef MOWGLI_DEBUG        
    ROS_INFO_STREAM("mowgli_proxy: GPSFixQualityCB");  
#endif    
    gps_quality_flags = msg->flags;    
}

/*
* GPS fix message received, extract coords and accurarcy#1, correct for datum
*/
void GPSPositionReceivedFixCB(const sensor_msgs::NavSatFix::ConstPtr &msg) 
{
    double n,e;
#ifdef MOWGLI_DEBUG        
    ROS_INFO_STREAM("mowgli_proxy: GPSPositionReceivedFixCB");
#endif    
    std::string zone;
    RobotLocalization::NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, n, e, zone);
    tf2::Vector3 gps_pos(
            (e-datumE), (n-datumN), 0.0
    );

    // Debug
    if (debug_gps_invalid)
    {   
          ROS_WARN_STREAM_THROTTLE(1, "mowgli_proxy: mowgli_debug/gps_invalid active - faking GPS outage");
          doDeadReckoning();
          return;
    }

    if(msg->position_covariance_type == 0) {
        ROS_WARN_STREAM_THROTTLE(1, "mowgli_proxy: Dropped GPS Update due to position_covariance_type being UNKNOWN");
        doDeadReckoning();
        return;
    }
    double acc_m = sqrt(msg->position_covariance[0]);
     if (acc_m > 0.05) {
        ROS_WARN_STREAM_THROTTLE(1, "mowgli_proxy: Dropping GPS update due to insufficient accuracy (>5cm): " << acc_m << "m");
        doDeadReckoning();
        return;
    }
    // test if we have a proper RTK fix accoring to /ublox/navpvt/flags
    unsigned int carrSoln = (gps_quality_flags >> 6) & 3;
    if (carrSoln != 0x2 && config.gpsrtk_fix_required)
    {
        ROS_WARN_STREAM_THROTTLE(1, "mowgli_proxy: Dropped GPS Update due to carrier phase range solution not being FIXED (gps quality might not be good enough)" << config.gpsrtk_fix_required);
        doDeadReckoning();
        gpsOdometryIsRTK = false;
        return;
    }    
    else
    {
        gpsOdometryIsRTK = true;
    } 
    handleGPSUpdate(gps_pos, acc_m);
}


/*
* IMU data (Madgwick filtered) - get orientation, and yaw
*/
void IMUDataCB(const sensor_msgs::Imu::ConstPtr &msg) {
#ifdef MOWGLI_DEBUG    
    ROS_INFO_STREAM("mowgli_proxy: IMUDataCB");
#endif    
  //  lastImu = *msg;
    hasImuMessage = true;

    // calculate Geometry Message Orientation from IMU Data
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;
    m.getRPY(unused1, unused2, yaw);

    // correct for rotated IMU
    yaw += config.imu_offset * (M_PI / 180.0);
    yaw = fmod(yaw + (M_PI_2), 2.0 * M_PI);
    while (yaw < 0) {
        yaw += M_PI * 2.0;
    }

    tf2::Quaternion q_mag;
    q_mag.setRPY(0.0, 0.0, yaw);
    r = yaw;

    mowgli_orientation = tf2::toMsg(q_mag);
#ifdef MOWGLI_DEBUG    
    ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: IMUDataCB yaw = %0.2f ", yaw*180/M_PI);
#endif    
}

void reconfigureCB(mowgli::MowgliProxyConfig &c, uint32_t level) 
{
    ROS_INFO_STREAM("mowgli_proxy: setting new MowgliProxy Configuration");
    config = c;
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mowgli_proxy");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ROS_INFO_STREAM("mowgli_proxy: Starting mowgli_proxy");

    // Dynamic reconfiguration server
    dynamic_reconfigure::Server<mowgli::MowgliProxyConfig> reconfig_server(paramNh);
    reconfig_server.setCallback(reconfigureCB);

    // Mowgli Topics
    subMowgliStatus = n.subscribe("mowgli/status", 50, MowgliStatusCB);    
    // subMowgliOdom = n.subscribe("mowgli/odom", 50, MowgliOdomCB);

    // IMU/Mag fusion topic (madgwick)
    subIMUData = n.subscribe("imu/data", 50, IMUDataCB);

    // Robot Localization EKF Output
    subEKFOdom = n.subscribe("odometry/filtered", 50, EKFOdomCB);

    // OpenMower Status
    pubOMStatus = n.advertise<mower_msgs::Status>("mower/status", 50);

    // GPS + Mowgli Odom fused
    pubOdom = n.advertise<nav_msgs::Odometry>("odom", 50);

    // DR
    subMowgliStatus = n.subscribe("mowgli/status", 50, MowgliStatusCB); 

    // DR EKF
    pubSetPose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 50);

    // Services required by OpenMower
    ros::ServiceServer om_emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
    ros::ServiceServer om_gps_service = n.advertiseService("mower_service/set_gps_state", setGpsState);

    // Debug Services
    ros::ServiceServer mowgli_debug_gpsonoff = n.advertiseService("mowgli_debug/gps_invalid", setDebugGPSInvalid);
    

    /*
     * GPS (ublox/fix) processing
     */
    bool gotLatLng = true;    
    gotLatLng &= paramNh.getParam("datum_lat", datumLat);
    gotLatLng &= paramNh.getParam("datum_long", datumLng);
    if(!gotLatLng) {
        ROS_ERROR_STREAM("mowgli_proxy: You need to provide a reference point if using lat/long coordinates for positioning! Set datum_lat and datum_long parameters");
        return 1;
    }
    RobotLocalization::NavsatConversions::LLtoUTM(datumLat, datumLng, datumN, datumE, datumZone);
    ROS_INFO_STREAM("mowgli_proxy: Odometry is using LAT/LONG positioning. Datum coordinates are: " << datumLat<< ", " << datumLng);
    subGPSPosFix = n.subscribe("ublox/fix", 100, GPSPositionReceivedFixCB);
    subGPSQuality = n.subscribe("ublox/navpvt", 100, GPSFixQualityCB);

    // Mowgli Services
    mowClient = n.serviceClient<std_srvs::SetBool>("mowgli/EnableMowerMotor");

    ROS_INFO("mowgli_blade: Waiting for mowgli/EnableMowerMotor server");
    if (!mowClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("mowgli_blade: EnableMowerMotor server service not found.");
        return 1;
    }

    ros::spin();
    return 0;
}
