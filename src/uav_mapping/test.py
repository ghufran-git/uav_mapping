#!/usr/bin/env python

from geographiclib.geodesic import Geodesic
import math

spot= [114,-36]
origin = [-36,-36]
resolution = 0.08 #meter/pixel
distancex = (spot[0] - origin[0])* resolution   # convert to meters
distancey = (spot[1] - origin[1]) * resolution  # convert to meters
origin_gps = [33.636864, 72.989602]

print("origin GPS : " + str(origin_gps))

distance = math.sqrt((distancex**2) + (distancey**2)) # Distance in meters (m)
print("distance from origin : " + str(distance) + " m ")

#check with coordinate axis is point
# find angle from origin 
# add this to 
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


# A = (33.00009, 74) #Point A (lat, lon)
# B = (33.63757044894407, 72.98968478657136) #Point B (lat, lon)
# s = 100 #Distance (m)

#Define the ellipsoid
geod = Geodesic.WGS84

#Solve the Inverse problem
# inv = geod.Inverse(A[0],A[1],B[0],B[1])
# azi1 = inv['azi1']
# print('Initial Azimuth from A to B = ' + str(azi1))
azi1= azimuth
#Solve the Direct problem
dir = geod.Direct(origin_gps[0],origin_gps[1],azi1,distance)
C = (dir['lat2'],dir['lon2'])
print('Goal GPS = ' + str(C))




