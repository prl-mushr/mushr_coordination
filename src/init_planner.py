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
    goal_count = 4
    pubs = []
    # this is basically initializing all the subscribers for counting the number of cars and publishers for initiailizing pose and goal points.
    for i in range(count):
        name = rospy.get_param("init_planner/car" + str(i+1) + "/name")
        print(name)
        publisher = rospy.Publisher(name+ "/init_pose", PoseStamped, queue_size=5)
        pubs.append(publisher)
    goal_pub = rospy.Publisher("/mushr_coordination/goals", GoalPoseArray, queue_size=5)
    obs_pub = rospy.Publisher("/mushr_coordination/obstacles", PoseArray, queue_size=5)
    rospy.sleep(1)

    car_pose = [[0, 10], [15, 3], [7, 5], [9, 5]]
    goal_pose = [[[6, 15], [8, 1]], [[12, 12], [7, 11]], [[10, 5], [5, 10]], [[5, 6], [15, 7]]]
    #car_pose = [[0, 0], [30, 30], [10, 20], [0, 30]]
    #goal_pose = [[[10, 10], [20, 20]], [[30, 10], [10, 30]], [[0, 10], [0, 20]], [[20, 0], [20, 30]]]

    for i in range(count):
        now = rospy.Time.now()
        carmsg = PoseStamped()
        carmsg.header.frame_id = "/map"
        carmsg.header.stamp = now
        carmsg.pose.position.x = car_pose[i][0]
        carmsg.pose.position.y = car_pose[i][1]
        carmsg.pose.position.z = 0.0
        #cur_pose.pose.orientation = angle_to_quaternion(rot)
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
    goalmsg.scale = 0.8
    goalmsg.minx = -2
    goalmsg.miny = -2
    goalmsg.maxx = 15
    goalmsg.maxy = 15
    for i in range(goal_count):
        goalmsg.goals.append(PoseArray())
        for j in range(2):
            goal = Pose()
            goal.position.x = goal_pose[i][j][0]
            goal.position.y = goal_pose[i][j][1]
            goal.position.z = 0.0
            goalmsg.goals[i].poses.append(goal)
    goal_pub.publish(goalmsg)
    #rospy.spin()
