#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped

rospy.init_node("pose_publisher_test", anonymous=True)

pub = rospy.Publisher("/pose_publisher", PoseStamped, queue_size=10)

robot_pose = PoseStamped()

robot_pose.pose.position.x = 1

robot_pose.pose.position.y = 2 

while not rospy.is_shutdown():
    pub.publish(robot_pose)
    rospy.sleep(5)


