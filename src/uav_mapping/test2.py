#transformation matrix 

import numpy as np
import math
bias = 450
x=0 +bias
y=50 +bias
z=0 
R= 0
P=0
Y=math.radians(0)
# ****************
x1=449.99846021
y1=450.0074232
z1=-0.00100139 
R1=0
P1=0
Y1=math.radians(0.0000001)
transformation= np.array([[round(math.cos(Y),4) , round(-math.sin(Y), 4), 0, x], [round(math.sin(Y),4), round(math.cos(Y), 4), 0, y],[0,0,1,z],[0,0,0,1]]) 
# translation = np.array([[x],[y],[z]])
print(transformation)
# u = np.array([[x1],[y1],[z1],[1]])
# print(u)
# u0= np.matmul(transformation,u)
# print("********************")
# print(u0)
# print(transformation[1][3])
# u1 = np.matmul(transformation,u0)
# print(u1)
# transformation1= np.array([[round(math.cos(Y1),4) , round(-math.sin(Y1), 4), 0, x1], [round(math.sin(Y1),4), round(math.cos(Y1), 4), 0, y1],[0,0,1,z1],[0,0,0,1]]) 
# u0=np.matmul(transformation,transformation1)

# print(u0)


#********************** Complete transformation with roll pitch yaw
roll=0
pitch=0
yaw=math.radians(0)
bias = 450
x=0 +bias
y=50 +bias
z=0 
#transformation = [[a,b,c,x],
#                  [d,e,f,y],
#                  [g,h,i,z]
#                  [0,0,0,1]]
a = round(math.cos(roll),4)*round(math.cos(pitch),4)
b = round(math.cos(roll),4)*round(math.sin(pitch),4)*round(math.sin(yaw),4)-round(math.sin(roll),4)*round(math.cos(yaw),4)
c = round(math.cos(roll),4)*round(math.sin(pitch),4)*round(math.cos(yaw),4) + round(math.sin(roll),4)*round(math.sin(yaw),4)
d = round(math.sin(roll),4)*round(math.cos(pitch),4)
e = round(math.sin(roll),4)*round(math.sin(pitch),4)*round(math.sin(yaw),4)+round(math.cos(roll),4)*round(math.cos(yaw),4)
f = round(math.sin(roll),4)*round(math.sin(pitch),4)*round(math.cos(yaw),4)-round(math.cos(roll),4)*round(math.sin(yaw),4)
g = - round(math.sin(pitch),4)
h = round(math.cos(pitch),4)*round(math.sin(yaw),4)
i = round(math.cos(pitch),4)*round(math.cos(yaw),4)

transformation=np.array([[a,b,c,x],[d,e,f,y],[g,h,i,z],[0,0,0,1]])
print("***************************")
print(transformation)
print("***************************")