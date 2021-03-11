#!/usr/bin/env python
""" 
Occupancy-grid-based mapping for drone(s). 

Subscribed topics:
(namespace)/mavros/local_position/odom

Published topics:
(namespace)/map

Author: Ghufran Ullah
Email: contact.ghufranullah@gmail.com
"""
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import numpy as np
import math
from geographiclib.geodesic import Geodesic

class UavMapper:
    """ 
        The UAVMapper class takes the values from odom and based on the position of the robot, 
        updates the map with the help of the _map object. 

    """

    def __init__(self):
        """ Construct an empty occupancy grid. Subscriber and publisher."""

        self.robot_pose = PoseStamped()
        self.position_sub = rospy.Subscriber(
            "mavros/local_position/odom", Odometry, self.callback_robot_pose, queue_size=1)
        self._map = Map()

    def callback_robot_pose(self, pose_recieved):
        """ Callback for /mavros/local_position/odom """

        self.robot_pose.pose.position.x = (
            pose_recieved.pose.pose.position.x - self._map.map_origin_x) * 1/self._map.map_resolution
        self.robot_pose.pose.position.y = (
            pose_recieved.pose.pose.position.y - self._map.map_origin_y) * 1/self._map.map_resolution
        self.robot_pose.pose.position.z = pose_recieved.pose.pose.position.z
        # print(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z)
        print(pose_recieved.pose.pose.position.x,pose_recieved.pose.pose.position.y,pose_recieved.pose.pose.position.z)
        if (self.robot_pose.pose.position.x <= 0 or self.robot_pose.pose.position.y <= 0):
            self.robot_pose.pose.position.x = 0
            self.robot_pose.pose.position.y = 0
            rospy.logerr("Out of Bounds of the map")
        self._map.mapUpdate(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z)


class Map:
    """
        The Map class stores an occupancy grid as a two dimensional numpy array.

        Variables accessed from uav_mapping/cfg/map_config.yaml. These are as follows:

        /uav_mapping/resolution     --  Resolution of the map, meters / pixel
        /uav_mapping/origin_x       --  Position of the x point in grid cell (lower-left pixel in the map)
        /uav_mapping/origin_y       --  Position of the y point in grid cell (lower-left pixel in the map)
        /uav_mapping/size_x         --  Number of columns in the occupancy grid.
        /uav_mapping/size_y         --  Number of rows in the occupancy grid.
        self.grid                   --  Numpy array

    """

    def __init__(self):
        self.map_resolution = rospy.get_param("/uav_mapping/resolution")
        self.map_origin_x = rospy.get_param("/uav_mapping/origin_x")
        self.map_origin_y = rospy.get_param("/uav_mapping/origin_y")
        self.map_size_x = rospy.get_param("/uav_mapping/size_x")
        self.map_size_y = rospy.get_param("/uav_mapping/size_y")
        self.map = OccupancyGrid()
        self.map.info.origin = Pose(
            Point(self.map_origin_x, self.map_origin_y, 0), Quaternion(0, 0, 0, 1))
        self.map.info.resolution = self.map_resolution
        self.map.info.width = self.map_size_x
        self.map.info.height = self.map_size_y
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=1, latch=True)
        self.pixel = 0
        self.grid = np.ones((self.map.info.height, self.map.info.width))
        self.grid = np.dot(self.grid, -1)
        flat_grid = self.grid.reshape((self.grid.size,))
        self.map.data = list(np.round(flat_grid))
        self.camera_fov = 40

    def publishMap(self):
        """ Publishes the map  """

        self.map_pub.publish(self.map)

    def mapUpdate(self, pos_x , pos_y, pos_z):
        """ Returns a nav_msgs/OccupancyGrid representation of the map (updated if any changes occured). """

        self.calculatePixel(pos_z)
        a = int(round(pos_x)) - int(round(self.pixel/2))
        if(a <= 0):
            a = 0
        b = int(round(pos_x)) + int(round(self.pixel/2))
        c = int(round(pos_y)) - int(round(self.pixel/2))
        if(c <= 0):
            c = 0
        d = int(round(pos_y)) + int(round(self.pixel/2))

        self.grid[a:b, c:d] = 1
        flat_grid = self.grid.reshape((self.grid.size,))
        self.map.data = list(np.round(flat_grid))

        return self.map

    def calculatePixel(self, pos_z):
        """ Calculates the pixels to mark as explored by the drone """

        radius = math.sqrt(((pos_z/math.cos(
            math.radians(self.camera_fov/2)))**2) - pos_z**2)
        self.pixel = (2*radius)/self.map_resolution

    def locateGps(self, pos_x, pos_y):
        spot= [pos_x, pox_y]
        origin = [self.map_origin_x,self.map_origin_x]
        distancex = (spot[0] - origin[0])* self.map_resolution  # convert to meters
        distancey = (spot[1] - origin[1])* self.map_resolution  # convert to meters
        origin_gps = [33.636864, 72.989602]

        print("origin GPS : " + str(origin_gps))

        distance = math.sqrt((distancex**2) + (distancey**2)) # Distance in meters (m)
        print("distance from origin : " + str(distance) + " m ")

        if (spot[0] > origin[0] and spot[1] > origin[1]):
            azimuth = math.degrees(math.atan(abs((spot[1]-origin[1])/(spot[0]-origin[0])))) #Q1
        elif (spot[0] < origin[0] and spot[1] >= origin[1]):
            azimuth = 270 + math.degrees(math.atan(abs((spot[1]-origin[1])/(spot[0]-origin[0])))) #Q2
        elif (spot[0] < origin[0] and spot[1] < origin[1]):
            azimuth = 270 - math.degrees(math.atan(abs((spot[1]-origin[1])/(spot[0]-origin[0])))) #Q3
        elif (spot[0] > origin[0] and spot[1] <= origin[1]):
            azimuth = 90 + math.degrees(math.atan(abs((spot[1]-origin[1])/(spot[0]-origin[0])))) #Q4
        elif (spot[0] == origin[0] and spot[1] >= origin[1]):
            azimuth = 0
        elif (spot[0] == origin[0] and spot[1] < origin[1]):
            azimuth = 180
        
        print("azimuth =" + str(azimuth))

        #Define the ellipsoid
        geod = Geodesic.WGS84
        azi1= azimuth

        #Solve the Direct problem
        dir = geod.Direct(origin_gps[0],origin_gps[1],azi1,distance)
        goal_gps = (dir['lat2'],dir['lon2'])
        print('Goal GPS = ' + str(goal_gps))
        return goal_gps


if __name__ == '__main__':
    rospy.init_node("Mapper", anonymous=True)
    test = UavMapper()
    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        print("publishing....")
        test._map.publishMap()
        r.sleep()
