#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
import time
import math
from mushr_coordination.msg import GoalPoseArray
from std_msgs.msg import String


def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


if __name__ == "__main__":
    rospy.init_node("init_planner")
    rospy.sleep(1)

    count = rospy.get_param("init_planner/num_agent")
    goal_count = 2
    pubs = []
    # this is basically initializing all the subscribers for counting the
    # number of cars and publishers for initiailizing pose and goal points.
    for i in range(count):
        name = rospy.get_param("init_planner/car" + str(i+1) + "/name")
        print(name)
        publisher = rospy.Publisher(
            name + "/init_pose", PoseStamped, queue_size=5)
        pubs.append(publisher)
    goal_pub = rospy.Publisher(
        "/mushr_coordination/goals", GoalPoseArray, queue_size=5)
    obs_pub = rospy.Publisher(
        "/mushr_coordination/obstacles", PoseArray, queue_size=5)
    rospy.sleep(1)

    car_pose = [[5, 2], [2.5, 3], [0, 2], [2.5, 0]]
    goal_pose = [[[1, 2], [4, 2]], [[0, 0], [3, 1]]]

    for i in range(count):
        now = rospy.Time.now()
        carmsg = PoseStamped()
        carmsg.header.frame_id = "/map"
        carmsg.header.stamp = now
        carmsg.pose.position.x = car_pose[i][0]
        carmsg.pose.position.y = car_pose[i][1]
        carmsg.pose.position.z = 0.0
        # cur_pose.pose.orientation = angle_to_quaternion(rot)
        print(carmsg)
        rospy.sleep(1)
        pubs[i].publish(carmsg)

    now = rospy.Time.now()
    obsmsg = PoseArray()
    obsmsg.header.frame_id = "/map"
    obsmsg.header.stamp = now
    obs_pub.publish(obsmsg)
    goalmsg = GoalPoseArray()
    goalmsg.header.frame_id = "/map"
    goalmsg.header.stamp = now
    goalmsg.scale = 1
    goalmsg.minx = 0
    goalmsg.miny = 0
    goalmsg.maxx = 5
    goalmsg.maxy = 3
    for i in range(goal_count):
        goalmsg.goals.append(PoseArray())
        for j in range(2):
            goal = Pose()
            goal.position.x = goal_pose[i][j][0]
            goal.position.y = goal_pose[i][j][1]
            goal.position.z = 0.0
            goalmsg.goals[i].poses.append(goal)
    goal_pub.publish(goalmsg)
    # rospy.spin()
