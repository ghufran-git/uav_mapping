#!/usr/bin/env python
""" Occupancy-grid-based mapping for a single drone. 

Subscribed topics:
/mavros/local_position/odom

Published topics:
test/map

Author: Ghufran Ullah
Email: contact.ghufranullah@gmail.com
"""
import rospy
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import Pose , PoseStamped , Point , Quaternion
import numpy as np
import math


class UavMapper:
    """ 
    The UavMapper class stores an occupancy grid as a two dimensional
    numpy array.
    
    Variables accessed from uav_mapping/cfg/map_config.yaml. These are as follows:

        /uav_mapping/resolution     --  Resolution of the map, meters / pixel
        /uav_mapping/origin_x       --  Position of the x point in grid cell (lower-left pixel in the map)
        /uav_mapping/origin_y       --  Position of the y point in grid cell (lower-left pixel in the map)
        /uav_mapping/size_x         --  Number of columns in the occupancy grid.
        /uav_mapping/size_y         --  Number of rows in the occupancy grid.
        self.grid                   --  numpy array 
        
    """
    def __init__(self):
        """ Construct an empty occupancy grid. Subscriber and publisher."""

        self.map_resolution = rospy.get_param("/uav_mapping/resolution")
        self.map_origin_x = rospy.get_param("/uav_mapping/origin_x")
        self.map_origin_y = rospy.get_param("/uav_mapping/origin_y")
        self.map_size_x = rospy.get_param("/uav_mapping/size_x")
        self.map_size_y = rospy.get_param("/uav_mapping/size_y")
        self.map = OccupancyGrid()
        self.map.info.origin = Pose(Point(self.map_origin_x, self.map_origin_y, 0), Quaternion(0,0,0,1))
        self.map.info.resolution = self.map_resolution
        self.map.info.width = self.map_size_x 
        self.map.info.height = self.map_size_y
        self.robot_pose = PoseStamped()
        self.map_pub = rospy.Publisher("test/map", OccupancyGrid, queue_size=1, latch=True)
        self.position_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.callback_robot_pose , queue_size=1)
        self.camera_fov = 40 
        self.pixel = 0
        self.grid = np.ones((self.map.info.height, self.map.info.width))
        self.grid = np.dot(self.grid , -1)
        flat_grid = self.grid.reshape((self.grid.size,))
        self.map.data = list(np.round(flat_grid))

    def callback_robot_pose(self, pose_recieved):
        """ Callback for /mavros/local_position/odom """

        self.robot_pose.pose.position.x = (pose_recieved.pose.pose.position.x - self.map_origin_x) * 1/self.map_resolution
        self.robot_pose.pose.position.y = (pose_recieved.pose.pose.position.y - self.map_origin_y) * 1/self.map_resolution
        self.robot_pose.pose.position.z = pose_recieved.pose.pose.position.z 
        print(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z)
        self.mapUpdate()


    def publishMap(self):
        """ Publishes the map  """

        self.map_pub.publish(self.map)

    def mapUpdate(self):
        """ Returns a nav_msgs/OccupancyGrid representation of the map (updated if any changes occured). """

        self.calculatePixel()
        self.grid[int(round(self.robot_pose.pose.position.x))-int(round(self.pixel/2)):int(round(self.robot_pose.pose.position.x))+int(round(self.pixel/2)),int(round(self.robot_pose.pose.position.y))-int(round(self.pixel/2)):int(round(self.robot_pose.pose.position.y))+int(round(self.pixel/2))] = 1
        flat_grid = self.grid.reshape((self.grid.size,))
        self.map.data = list(np.round(flat_grid))

        return self.map

    def calculatePixel(self):
        """ Calculates the pixels to mark as explored by the drone """

        radius = math.sqrt(((self.robot_pose.pose.position.z/math.cos(math.radians(self.camera_fov/2)))**2) - self.robot_pose.pose.position.z**2)
        self.pixel = (2*radius)/self.map_resolution

if __name__ == '__main__':
    rospy.init_node("UavMapper", anonymous=True)
    test = UavMapper()
    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        print("publishing...")
        test.publishMap()
        r.sleep()
