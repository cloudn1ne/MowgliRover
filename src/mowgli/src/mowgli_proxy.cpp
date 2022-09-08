/*
 * Mowgli to OpenMower Proxy V 1.0 
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/Mowgli
 *
 *
 * v1.0: inital release
 *
 */

#include "ros/ros.h"

/* OM */
#include <mower_msgs/Status.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/MowerControlSrv.h>
#include <mower_msgs/GPSControlSrv.h>
#include <mower_msgs/EmergencyStopSrv.h>

/* Mowgli */
#include "std_srvs/SetBool.h"
#include <mowgli/MowgliProxyConfig.h>
#include <mowgli/status.h>


/* GPS/ODOM */
#include <nav_msgs/Odometry.h>
#include <ublox_msgs/NavRELPOSNED9.h>

/* TF */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <robot_localization/navsat_conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>


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
int gps_outlier_count = 0;
int valid_gps_samples = 0;
bool gpsOdometryValid = false;
bool gpsEnabled = true;


// IMU
//sensor_msgs::Imu lastImu;
bool hasImuMessage = false;
geometry_msgs::Quaternion mowgli_orientation;


// ODOM
double x = 0, y = 0, r = 0;
double vx = 0, vy = 0, vr = 0;

// Pubs
ros::Publisher pubOMStatus;
ros::Publisher pubOdom;

// Subs
ros::Subscriber subGPSPosFix;
ros::Subscriber subIMUData;
ros::Subscriber subMowgliStatus;
ros::Subscriber subMowgliOdom;

// Service Clients
ros::ServiceClient mowClient;
bool mowEnabledFlag = false;
ros::Time last_mowEnabledFlagSent(0.0);

// TF
//tf2_ros::Buffer tfBuffer; 
//tf::TransformBroadcaster transform_broadcaster;
//tf2_ros::TransformListener tfListener(tfBuffer);
//geometry_msgs::TransformStamped transformStamped;

bool setGpsState(mower_msgs::GPSControlSrvRequest &req, mower_msgs::GPSControlSrvResponse &res) {
    ROS_INFO_STREAM("mowgli_proxy: setGpsState =" << gpsEnabled);
    gpsEnabled = req.gps_enabled;
    gpsOdometryValid = false;    
    return true;
}


// tell mowgli to enable/disable mower
void sendMowEnabled(bool ena_dis)
{
    std_srvs::SetBool mow_srv;
    mow_srv.request.data = ena_dis;
    ROS_INFO("-------------------------------------");
    ROS_INFO("mowgli_proxy: setMowEnabled = %d ", ena_dis);
    mowClient.call(mow_srv);
    ROS_INFO("-------------------------------------");
    mowEnabledFlag = ena_dis;
    last_mowEnabledFlagSent = ros::Time::now();
}

// state machine to enable/disable blade in mowgli because we need to send keepalives
// or mowgli will stop the blade after 25secs - service calls are heavy on rosserial - so we only send when needed
bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
//    config.mower_running = req.mow_enabled;
//    reconfig_server->updateConfig(config);

    // ROS_INFO("mowgli_proxy: setMowEnabled = %d", req.mow_enabled);
    // DISABLED -> ENABLED
    if (req.mow_enabled && !mowEnabledFlag)
    {
        ROS_WARN_STREAM("Blade DISABLED -> ENABLED");
        sendMowEnabled(1);
    }
    // ENABLED -> ENABLED (keep alive interval 10 sec)
    else if (req.mow_enabled && mowEnabledFlag && (ros::Time::now() - last_mowEnabledFlagSent > ros::Duration(10)) )
    {
        ROS_WARN_STREAM("Blade ENABLED KEEP ALIVE");
        sendMowEnabled(1);
    }
    // ENABLED -> DISABLED
    else if (!req.mow_enabled && mowEnabledFlag)
    {
        ROS_WARN_STREAM("Blade ENABLED -> DISABLED");
        sendMowEnabled(0);
    }
    return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
//    config.emergency_stop = req.emergency;
//    reconfig_server->updateConfig(config);
    ROS_INFO_STREAM("mowgli_proxy: setEmergencyStop");
    return true;
}

/*
 * Publish Odometry and odom-base_link transform
 */ 
void pubOdometry() {

    // ROS_INFO_STREAM("mowgli_proxy: pubOdometry");
    static tf2_ros::TransformBroadcaster transform_broadcaster;
    
    ros::Time current_time = ros::Time::now();

    /* Transform */
    geometry_msgs::TransformStamped odom_trans;    
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    // TODO: Add logic for 3d odometry
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = mowgli_orientation;
    transform_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    // set the position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_trans.transform.rotation;
    // set the velocity    
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = vr;

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

    if (gpsOdometryValid && gpsEnabled && (ros::Time::now() - last_gps_odometry_time) < ros::Duration(5.0)) {
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


    ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: pubOdometry x = %0.2f y = %0.2f", x, y);

    // publish the message
    pubOdom.publish(odom_msg);
}


void handleGPSUpdate(tf2::Vector3 gps_pos, double gps_accuracy_m) {

    if (gps_accuracy_m > 0.05) {
        ROS_INFO_STREAM("mowgli_proxy: Dropping GPS update due to insufficient accuracy (>5cm): " << gps_accuracy_m << "m");
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_odometry_time).toSec();

    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("mowgli_proxy: last GPS was received " << time_since_last_gps << " seconds ago.");
        gpsOdometryValid = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps_pos = gps_pos;
        last_gps_acc_m = gps_accuracy_m;
        last_gps_odometry_time = ros::Time::now();
        return;
    }


    //    ROS_INFO_STREAM("GOT GPS: " << gps_pos.x() << ", " << gps_pos.y());
    double distance_to_last_gps = (last_gps_pos - gps_pos).length();
   // ROS_INFO_STREAM("mowgli_proxy: distance_to_last_gps = " << distance_to_last_gps );
    if (distance_to_last_gps < 5.0) {
        // inlier, we treat it normally
        // calculate current base_link position from orientation and distance parameter

        double base_link_x;
        double base_link_y;

        base_link_x = gps_pos.x() - config.gps_antenna_offset * cos(r);
        base_link_y = gps_pos.y() - config.gps_antenna_offset * sin(r);
        //ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: gps_pos.x/y   x = %0.2f y = %0.2f", gps_pos.x(), gps_pos.y());
        //ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: base_link_x/y x = %0.2f y = %0.2f", base_link_x, base_link_y);

    
        // store the gps as last
        last_gps_pos = gps_pos;
        last_gps_acc_m = gps_accuracy_m;
        last_gps_odometry_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;
        if (!gpsOdometryValid && valid_gps_samples > 10) {
            ROS_INFO_STREAM("mowgli_proxy:: received 10 valid GPS samples, odometry is now valid");
            ROS_INFO_STREAM("mowgli_proxy: new coords are: " << base_link_x << ", " << base_link_y);
            // we don't even have gps yet, set odometry to first estimate
            x = base_link_x;
            y = base_link_y;
            gpsOdometryValid = true;
        } else if (gpsOdometryValid) {
            // gps was valid before, we apply the filter
            x = x * (1.0 - config.gps_filter_factor) + config.gps_filter_factor * base_link_x;
            y = y * (1.0 - config.gps_filter_factor) + config.gps_filter_factor * base_link_y;
        }
    } else {
        ROS_WARN_STREAM("mowgli_proxy: new GPS data is more than 5cm away from last fix. the calculated distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) {
            ROS_ERROR_STREAM("mowgli_proxy: too many GPS outliers, assuming that the current gps value is invalid.");
            last_gps_pos = gps_pos;
            last_gps_acc_m = gps_accuracy_m;
            last_gps_odometry_time = ros::Time::now();
            gpsOdometryValid = false;
            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }
    pubOdometry();
}

void MowgliOdomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO_STREAM("mowgli_proxy: MowgliOdomCB");

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    vx = msg->twist.twist.linear.x;
    vy = 0;
    vr = msg->twist.twist.angular.z;
  //  ROS_INFO_DELAYED_THROTTLE(1, "mowgli_proxy: MowgliOdomCB x = %0.2f y = %0.2f", x, y);
}

/*
* translate /mowgli/status to /mower/status
*/
void MowgliStatusCB(const mowgli::status::ConstPtr &msg) 
{
    mower_msgs::Status om_mower_status;

    // ROS_INFO_STREAM("mowgli_proxy: MowgliStatusCB");

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

    pubOMStatus.publish(om_mower_status);
}

void GPSPositionReceivedFixCB(const sensor_msgs::NavSatFix::ConstPtr &msg) 
{
    double n,e;
    
    //ROS_INFO_STREAM("mowgli_proxy: GPSPositionReceivedFixCB");
    std::string zone;
    RobotLocalization::NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, n, e, zone);
    tf2::Vector3 gps_pos(
            (e-datumE), (n-datumN), 0.0
    );

    if(msg->position_covariance_type == 0) {
        ROS_INFO_STREAM_THROTTLE(1, "mowgli_proxy: Dropped GPS Update due to position_covariance_type being UNKNOWN");
        return;
    }

    double acc_m = sqrt(msg->position_covariance[0]);
    handleGPSUpdate(gps_pos, acc_m);
}


void IMUDataCB(const sensor_msgs::Imu::ConstPtr &msg) {

 //   ROS_INFO_STREAM("mowgli_proxy: IMUDataCB");
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
    ROS_INFO_DELAYED_THROTTLE(5, "mowgli_proxy: IMUDataCB yaw = %0.2f ", yaw*180/M_PI);
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

    ROS_INFO_STREAM("Starting mowgli_proxy");

    // Dynamic reconfiguration server
    dynamic_reconfigure::Server<mowgli::MowgliProxyConfig> reconfig_server(paramNh);
    reconfig_server.setCallback(reconfigureCB);

    // Mowgli Topics
    subMowgliStatus = n.subscribe("mowgli/status", 50, MowgliStatusCB);    
    subMowgliOdom = n.subscribe("mowgli/odom", 50, MowgliOdomCB);

    // Mowgli Services
    mowClient = n.serviceClient<std_srvs::SetBool>("mowgli/EnableMowerMotor");

    // IMU/Mag fusion topic (madgwick)
    subIMUData = n.subscribe("imu/data", 50, IMUDataCB);

    // OpenMower Status
    pubOMStatus = n.advertise<mower_msgs::Status>("mower/status", 50);

    // GPS + Mowgli Odom fused
    pubOdom = n.advertise<nav_msgs::Odometry>("odom", 50);


    // services required by OpenMower
    ros::ServiceServer om_emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
    ros::ServiceServer om_mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer om_gps_service = n.advertiseService("mower_service/set_gps_state", setGpsState);

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
    ROS_INFO_STREAM("Odometry is using LAT/LONG positioning. Datum coordinates are: " << datumLat<< ", " << datumLng);
    subGPSPosFix = n.subscribe("ublox/fix", 100, GPSPositionReceivedFixCB);


    ROS_INFO("Waiting for mowgli/EnableMowerMotor server");
    if (!mowClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("EnableMowerMotor server service not found.");
        return 1;
    }

    ros::spin();
    return 0;
}
