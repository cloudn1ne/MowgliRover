#!/usr/bin/env python3
import rospy
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu

imu_yaw = 0.0
odom_yaw = 0.0
mag_heading = 0.0

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def callback_odom(data):
    global odom_yaw

    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0] * 180/math.pi
    pitch = euler[1] * 180/math.pi
    odom_yaw = euler[2] * 180/math.pi

    #print("---------------------------------------------")
    #print("ODOM: yaw: %.3f " % (odom_yaw))
        
def callback_imu(data):    
    global imu_yaw

    quaternion = (
    data.orientation.x,
    data.orientation.y,
    data.orientation.z,
    data.orientation.w)
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0] * 180/math.pi
    pitch = euler[1] * 180/math.pi
    imu_yaw = euler[2] * 180/math.pi

    #print("---------------------------------------------")    
    #print("IMU yaw: %.3f " % (imu_yaw))

def callback_mag(data):    
    global mag_heading

    x = data.magnetic_field.x
    y = data.magnetic_field.y
    z = data.magnetic_field.x
      
    yxHeading = math.atan2(x, y);
    zxHeading = math.atan2(z, x);
    heading = yxHeading;
    declinationAngle = 0.0861;
    heading =  heading + declinationAngle;
    if (heading < 0):
        heading += 2*math.pi;
    if (heading > 2*math.pi):
        heading -= 2*math.pi;

    mag_heading = heading * 180/math.pi; 

   # print("---------------------------------------------")    
   # print("MAG heading: %.3f " % (mag_heading))

def main():
    mag_heading_offset = 0
    odom_yaw_offset = 0
    imu_yaw_offset = 0

    rospy.init_node('calibrate', anonymous=True)
    rospy.Subscriber("/imu/mag", MagneticField, callback_mag)
    #rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/odometry/filtered", Odometry, callback_odom)
    rospy.Subscriber("/imu/data_dr", Imu, callback_imu)
    #rospy.spin()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
       # first msg will be used as offset
       if mag_heading != 0 and mag_heading_offset == 0:
         mag_heading_offset = mag_heading
         print("MAG captured")
  
       if odom_yaw != 0 and odom_yaw_offset == 0:
         odom_yaw_offset = odom_yaw
         print("ODOM captured")

       if imu_yaw != 0 and imu_yaw_offset == 0:
         imu_yaw_offset = imu_yaw
         print("IMU captured")

       m = mag_heading - mag_heading_offset
       o = odom_yaw - odom_yaw_offset
       i = imu_yaw - imu_yaw_offset 
       print("MAG: %.3f \tODOM: %.3f \tIMU: %.3f" % (m,o,i))
       rate.sleep()

if __name__ == '__main__':
    main()
