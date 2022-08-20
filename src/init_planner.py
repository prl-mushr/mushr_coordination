#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
import time
import math
from mushr_coordination.msg import GoalPoseArray
from std_msgs.msg import String
import random
from copy import deepcopy

use_mocap_start = False  # set to true to use the mocap position as the start position, false to use the testcase start position

INVALID_POSE_Z = -100.0
mocap_poses = []

def mocap_callback(p, i):
    mocap_poses[i] = p

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


if __name__ == "__main__":
    rospy.init_node("init_planner")
    rospy.sleep(1)

    num_agent = rospy.get_param("init_clcbs/num_agent")
    num_task = rospy.get_param("init_clcbs/num_task")
    num_waypoint = rospy.get_param("init_clcbs/num_waypoint")
    randomness = rospy.get_param("init_clcbs/randomness")
    x_min = rospy.get_param("init_clcbs/minx")
    x_max = rospy.get_param("init_clcbs/maxx")
    y_min = rospy.get_param("init_clcbs/miny")
    y_max = rospy.get_param("init_clcbs/maxy")
    pubs = []
    mocap_subs = []
    # this is basically initializing all the subscribers for counting the number of cars and publishers for initiailizing pose and goal points.
    for i in range(num_agent):
        name = rospy.get_param("init_clcbs/car" + str(i+1) + "/name")
        publisher = rospy.Publisher(name + "/init_pose", PoseStamped, queue_size=5)
        pubs.append(publisher)
        if use_mocap_start:
            mocap_poses.append(PoseStamped())
            mocap_poses[i].pose.position.z = INVALID_POSE_Z
            mocap_sub = rospy.Subscriber("/" + name + "/car_pose", PoseStamped, callback=mocap_callback, callback_args=i)
            mocap_subs.append(mocap_sub)
    goal_pub = rospy.Publisher("/mushr_coordination/goals", GoalPoseArray, queue_size=5)
    obs_pub = rospy.Publisher("/mushr_coordination/obstacles", PoseArray, queue_size=5)
    clcbs_obs_pub = rospy.Publisher("/clcbs_ros/obstacles", PoseArray, queue_size=5)
    rospy.sleep(1)

    for i in range(num_agent):
        now = rospy.Time.now()
        carmsg = PoseStamped()
        carmsg.header.frame_id = "/map"
        carmsg.header.stamp = now
        if use_mocap_start and mocap_poses[i].pose.position.z != INVALID_POSE_Z:  # make sure mocap has published for this agent
            carmsg.pose = deepcopy(mocap_poses[i].pose)
            carmsg.pose.position.z = 0.0
        else:
            carmsg.pose.position.x = min(x_max, max(x_min, rospy.get_param("init_clcbs/car" + str(i + 1) + "/x") + random.uniform(-randomness[0], randomness[0])))
            carmsg.pose.position.y = min(y_max, max(y_min, rospy.get_param("init_clcbs/car" + str(i + 1) + "/y") + random.uniform(-randomness[1], randomness[1])))
            carmsg.pose.position.z = 0.0
            carmsg.pose.orientation = angle_to_quaternion(rospy.get_param("init_clcbs/car" + str(i + 1) + "/t"))
        print(carmsg)
        rospy.sleep(1)
        pubs[i].publish(carmsg)

    now = rospy.Time.now()
    obsmsg = PoseArray()
    obsmsg.header.frame_id = "/map"
    obsmsg.header.stamp = now
    obs_pub.publish(obsmsg)
    clcbs_obs_pub.publish(obsmsg)
    goalmsg = GoalPoseArray()
    goalmsg.header.frame_id = "/map"
    goalmsg.header.stamp = now
    goalmsg.scale = rospy.get_param("init_clcbs/scale")
    goalmsg.minx = x_min
    goalmsg.miny = y_min
    goalmsg.maxx = x_max
    goalmsg.maxy = y_max
    for i in range(num_task):
        goalmsg.goals.append(PoseArray())
        for j in range(num_waypoint):
            goal = Pose()
            goal.position.x = rospy.get_param("init_clcbs/task" + str(i + 1) + "/p" + str(j + 1) + "/x")
            goal.position.y = rospy.get_param("init_clcbs/task" + str(i + 1) + "/p" + str(j + 1) + "/y")
            goal.position.z = 0.0
            goal.orientation = angle_to_quaternion(rospy.get_param("init_clcbs/task" + str(i + 1) + "/p" + str(j + 1) + "/t"))
            goalmsg.goals[i].poses.append(goal)
    goal_pub.publish(goalmsg)
    #rospy.spin()
