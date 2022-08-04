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

    num_agent = rospy.get_param("init_clcbs/num_agent")
    num_task = rospy.get_param("init_clcbs/num_task")
    num_waypoint = rospy.get_param("init_clcbs/num_waypoint")
    pubs = []
    # this is basically initializing all the subscribers for counting the number of cars and publishers for initiailizing pose and goal points.
    for i in range(num_agent):
        name = rospy.get_param("init_clcbs/car" + str(i+1) + "/name")
        publisher = rospy.Publisher(name + "/init_pose", PoseStamped, queue_size=5)
        pubs.append(publisher)
    goal_pub = rospy.Publisher("/mushr_coordination/goals", GoalPoseArray, queue_size=5)
    obs_pub = rospy.Publisher("/mushr_coordination/obstacles", PoseArray, queue_size=5)
    clcbs_obs_pub = rospy.Publisher("/clcbs_ros/obstacles", PoseArray, queue_size=5)
    rospy.sleep(1)

    for i in range(num_agent):
        now = rospy.Time.now()
        carmsg = PoseStamped()
        carmsg.header.frame_id = "/map"
        carmsg.header.stamp = now
        carmsg.pose.position.x = rospy.get_param("init_clcbs/car" + str(i + 1) + "/x")
        carmsg.pose.position.y = rospy.get_param("init_clcbs/car" + str(i + 1) + "/y")
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
    goalmsg.minx = rospy.get_param("init_clcbs/minx")
    goalmsg.miny = rospy.get_param("init_clcbs/miny")
    goalmsg.maxx = rospy.get_param("init_clcbs/maxx")
    goalmsg.maxy = rospy.get_param("init_clcbs/maxy")
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
