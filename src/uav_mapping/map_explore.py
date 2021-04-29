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
import geometry_msgs.msg
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import numpy as np
import math
from geographiclib.geodesic import Geodesic
import tf_conversions
import tf2_ros

class UavMapper:
    """ 
        The UAVMapper class takes the values from odom and based on the position of the robot, 
        updates the map with the help of the _map object. 

    """

    def __init__(self):
        """ Construct an empty occupancy grid. Subscriber and publisher."""

        self.ns = rospy.get_namespace()
        self.initial_x = rospy.get_param(self.ns + "/x")
        self.initial_y = rospy.get_param(self.ns + "/y")
        self.initial_z = rospy.get_param(self.ns + "/z")
        self.initial_R = rospy.get_param(self.ns + "/R")
        self.initial_P = rospy.get_param(self.ns + "/P")
        self.initial_Y = rospy.get_param(self.ns + "/Y")

        # self.initial_x = 0
        # self.initial_y = 0
        # self.initial_z = 0
        # self.initial_R = 0
        # self.initial_P = 0
        # self.initial_Y = 0
        self.robot_pose = PoseStamped()
  
        self._map = Map()
        
        self.robot_pose.pose.position.x=(self.initial_x - self._map.map_origin_x) * 1/self._map.map_resolution
        self.robot_pose.pose.position.y=(self.initial_y - self._map.map_origin_x) * 1/self._map.map_resolution
        self.robot_pose.pose.position.z= self.initial_z

        self.robot_roll=math.radians(self.initial_R)
        self.robot_pitch=math.radians(self.initial_P)
        self.robot_yaw=math.radians(self.initial_Y)
        
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        self.robot_transformation= geometry_msgs.msg.TransformStamped()
        self.robot_transformation.header.stamp = rospy.Time.now()
        self.robot_transformation.header.frame_id = "map"
        self.robot_transformation.child_frame_id = str(self.ns) + "base_link_offset"
        print(self.robot_transformation.header.frame_id, self.robot_transformation.child_frame_id)

        self.initial_transform= self.transformation()
        self.new_transform=np.identity(4)

        self.position_sub = rospy.Subscriber(
            "mavros/local_position/odom", Odometry, self.callback_robot_pose, queue_size=1)

    def callback_robot_pose(self, pose_recieved):
        """ Callback for /mavros/local_position/odom """

        self.robot_pose.pose.position.x = pose_recieved.pose.pose.position.x
        self.robot_pose.pose.position.y = pose_recieved.pose.pose.position.y 
        self.robot_pose.pose.position.z = pose_recieved.pose.pose.position.z
        self.send_transforms()
        # print(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z)
        # print(pose_recieved.pose.pose.position.x,pose_recieved.pose.pose.position.y,pose_recieved.pose.pose.position.z)
        # if (self.robot_pose.pose.position.x <= 0 or self.robot_pose.pose.position.y <= 0):
        #     self.robot_pose.pose.position.x = 0
        #     self.robot_pose.pose.position.y = 0
        #     rospy.logerr("Out of Bounds of the map")
        
        self.robot_roll , self.robot_pitch , self.robot_yaw = self.euler_from_quaternion(self.robot_pose.pose.orientation.x, self.robot_pose.pose.orientation.y, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w)
        
        self.new_transform= self.transformation()
        # self.new_transform= np.array([[self.robot_pose.pose.position.x], [self.robot_pose.pose.position.y], [self.robot_pose.pose.position.z],[1]])
        test = np.matmul(self.initial_transform, self.new_transform)
        self.robot_pose.pose.position.x=test[0][3] 
        self.robot_pose.pose.position.y=test[1][3] 
        self.robot_pose.pose.position.z=test[2][3] 
        # print(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z)
        
        self._map.mapUpdate(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z)

    def euler_from_quaternion(self, x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians

    def transformation(self):
        a = round(math.cos(self.robot_roll),4)*round(math.cos(self.robot_pitch),4)
        b = round(math.cos(self.robot_roll),4)*round(math.sin(self.robot_pitch),4)*round(math.sin(self.robot_yaw),4)-round(math.sin(self.robot_roll),4)*round(math.cos(self.robot_yaw),4)
        c = round(math.cos(self.robot_roll),4)*round(math.sin(self.robot_pitch),4)*round(math.cos(self.robot_yaw),4) + round(math.sin(self.robot_roll),4)*round(math.sin(self.robot_yaw),4)
        d = round(math.sin(self.robot_roll),4)*round(math.cos(self.robot_pitch),4)
        e = round(math.sin(self.robot_roll),4)*round(math.sin(self.robot_pitch),4)*round(math.sin(self.robot_yaw),4)+round(math.cos(self.robot_roll),4)*round(math.cos(self.robot_yaw),4)
        f = round(math.sin(self.robot_roll),4)*round(math.sin(self.robot_pitch),4)*round(math.cos(self.robot_yaw),4)-round(math.cos(self.robot_roll),4)*round(math.sin(self.robot_yaw),4)
        g = - round(math.sin(self.robot_pitch),4)
        h = round(math.cos(self.robot_pitch),4)*round(math.sin(self.robot_yaw),4)
        i = round(math.cos(self.robot_pitch),4)*round(math.cos(self.robot_yaw),4)
        x = self.robot_pose.pose.position.x
        y = self.robot_pose.pose.position.y
        z = self.robot_pose.pose.position.z
        transformation=np.array([[a,b,c,x],[d,e,f,y],[g,h,i,z],[0,0,0,1]])
        return transformation
    
    def send_transforms(self):
        self.robot_transformation.transform.translation.x = self.robot_pose.pose.position.x + self.initial_x
        self.robot_transformation.transform.translation.y = self.robot_pose.pose.position.y + self.initial_y
        self.robot_transformation.transform.translation.z = self.robot_pose.pose.position.z + self.initial_z

        self.robot_transformation.transform.rotation.x = 0
        self.robot_transformation.transform.rotation.y = 0
        self.robot_transformation.transform.rotation.z = 0
        self.robot_transformation.transform.rotation.w = 1
        self.transform_broadcaster.sendTransform(self.robot_transformation)


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
        a = int(round(pos_y)) - int(round(self.pixel/2))
        if(a <= 0):
            a = 0
        b = int(round(pos_y)) + int(round(self.pixel/2))
        c = int(round(pos_x)) - int(round(self.pixel/2))
        if(c <= 0):
            c = 0
        d = int(round(pos_x)) + int(round(self.pixel/2))

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
        print( str(test.ns) + "  Publishing....")
        test._map.publishMap()
        r.sleep()
