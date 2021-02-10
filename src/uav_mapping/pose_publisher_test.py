#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry

rospy.init_node("pose_publisher_test", anonymous=True)

pub = rospy.Publisher("/mavros/local_position/odom", Odometry, queue_size=10)

robot_pose = Odometry()



while not rospy.is_shutdown():
    for i in range(1000):
        if(rospy.is_shutdown()):
            break
        robot_pose.pose.pose.position.x = i
        robot_pose.pose.pose.position.y = i+1
        print(robot_pose)
        pub.publish(robot_pose)
        rospy.sleep(0.1)


