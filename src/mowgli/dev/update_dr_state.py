#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

initpose = PoseWithCovarianceStamped()
pose_captured = False
pose_set = False

def mowgli_odom(data):
    global initpose, pose_captured

   # rospy.loginfo("Init DR: /odom -> x = %0.2f y = %0.2f", data.pose.pose.position.x, data.pose.pose.position.y)
    initpose.header.stamp = rospy.get_rostime()
    initpose.header.frame_id = "map"
    initpose.pose.pose = data.pose.pose
    initpose.pose.covariance = [1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
       1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
       1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9]
    pose_captured = True
    


if __name__ == '__main__':
    try:
        rospy.init_node('update_dr_state')
        rospy.Subscriber("/odom", Odometry, mowgli_odom)
        initialpose_pub = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)

        while not rospy.is_shutdown() and not pose_set:
            if pose_captured:
                rospy.loginfo("calling /set_pose with x = %0.2f y = %0.2f", initpose.pose.pose.position.x, initpose.pose.pose.position.y)
                initialpose_pub.publish(initpose)
                #pose_set = True
            rospy.sleep(15.0)

    except rospy.ROSInterruptException:
        rospy.loginfo("Initial Pose for DR finished.")
