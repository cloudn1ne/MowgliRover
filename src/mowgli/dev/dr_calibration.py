#!/usr/bin/env python3
#
# Dead Reckoning Calibration Test
# (c) Georg Swoboda <cn@warp.at> 2022
#
# https://github.com/cloudn1ne/MowgliRover
#
# v1.0: inital release
#

import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs
from tf.transformations import quaternion_from_euler
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from mowgli.msg import status

# mowgli odom
mowgli_x = 0
mowgli_y = 0
# mowgli ticks
mowgli_left_encoder_ticks = 0
mowgli_right_encoder_ticks = 0
mowgli_left_encoder_ticks_offset = 0
mowgli_right_encoder_ticks_offset = 0


def dr_left_ticks():
    global mowgli_left_encoder_ticks, mowgli_left_encoder_ticks_offset
    return (mowgli_left_encoder_ticks-mowgli_left_encoder_ticks_offset)

def dr_right_ticks():
    global mowgli_right_encoder_ticks, mowgli_right_encoder_ticks_offset
    return (mowgli_right_encoder_ticks-mowgli_right_encoder_ticks_offset)

# sync dr odom with gps
def dr_reset():
    global mowgli_left_encoder_ticks_offset, mowgli_right_encoder_ticks_offset
    global mowgli_left_encoder_ticks, mowgli_right_encoder_ticks

    # reset tick offsets
    mowgli_left_encoder_ticks_offset = mowgli_left_encoder_ticks;
    mowgli_right_encoder_ticks_offset = mowgli_right_encoder_ticks;

   # rospy.wait_for_service('/mowgli/dr_reset')
   # try:
   #     rospy.loginfo("Resetting DR Offsets")
   #     dr_reset = rospy.ServiceProxy('/mowgli/dr_reset', SetBool)
   #     resp1 = dr_reset(1)
   #     return resp1
   # except rospy.ServiceException as e:
   #     print("Service call failed: %s"%e)

def mowgli_status(data):
    global mowgli_left_encoder_ticks, mowgli_right_encoder_ticks
    #rospy.loginfo("mowgli_status")
    mowgli_left_encoder_ticks= data.left_encoder_ticks
    mowgli_right_encoder_ticks = data.right_encoder_ticks

def mowgli_odom(data):
    global mowgli_x, mowgli_y
    #rospy.loginfo("mowgli_odom")
    mowgli_x = data.pose.pose.position.x
    mowgli_y = data.pose.pose.position.y

def create_goal(x, y, z, yaw_angle, planner):
    goal = mbf_msgs.MoveBaseGoal()
    goal.controller = planner
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z

    # Pose Orientation
    quaternion = quaternion_from_euler(0, 0, yaw_angle)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    return goal


def move(goal):
    mbf_ac.send_goal(goal)
    mbf_ac.wait_for_result()
    return mbf_ac.get_result()

# drive goals (once) - return tpm value
def drive_cycle(goals):
    r_idx = 0  
    for goal in goals:
        r_idx = r_idx + 1
        rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        result = move(goal)
        if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            rospy.loginfo("Unable to complete action: %s", result.message)
            return 
        if r_idx == 2:
            dr_reset()
        rospy.loginfo("mowgli is now at (%1.3f, %1.3f)", mowgli_x, mowgli_y)
        if r_idx == 3:
            tpm = ((dr_left_ticks()+dr_right_ticks())/2.0)/5.0      # avg l&r divided by distance (5.0)
            rospy.loginfo("mowgli ticks are now (%ld, %ld) ticks per meter: %0.2f", dr_left_ticks(), dr_right_ticks(), tpm)
            rospy.loginfo("----------------------------------------------------")
            return tpm

if __name__ == '__main__':
    rospy.init_node("dr_calibration_client")

    #rospy.Subscriber("/mowgli/odom", Odometry, mowgli_odom)
    rospy.Subscriber("/mowgli/status", status, mowgli_status)

    mbf_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_ac.wait_for_server(rospy.Duration(10))
    rospy.loginfo("Connected to Move Base Flex action server!")

    # drive cycle paths (horiz/vert)
    goals_x = [   create_goal(-6, -0, 0, 270, "CalibrationPlanner"), # approach position (use Fast Mode ;-)
                  create_goal(-6, -1, 0, 270, "CalibrationPlanner"), # start / reset ticks
                  create_goal(-6, -6, 0, 270, "CalibrationPlanner")  # final
    ]

    goals_y = [   create_goal(-9, -3, 0, 0, "CalibrationPlanner"), # approach position (use Fast Mode ;-)
                  create_goal(-8, -3, 0, 0, "CalibrationPlanner"), # start / reset ticks
                  create_goal(-3, -3, 0, 0, "CalibrationPlanner")  # final
    ]

    cycles = 1      # how many X and Y paths should be driven and averaged
    tpm_sum = 0
    for x in range(cycles):
        tpm_sum = tpm_sum + drive_cycle(goals_x)
        tpm_sum = tpm_sum + drive_cycle(goals_y)

    tpm = tpm_sum / (cycles*2)
    rospy.loginfo("overall averaged ticks per meter: %0.2f", tpm)
