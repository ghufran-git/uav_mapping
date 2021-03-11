#transformation matrix 

import numpy as np
import math
x=0
y=20
z=0
R= 0
P=0
Y=math.radians(180)
# ****************
x1=2
y1=4
z1=0
R1=0
P1=0
Y1=math.radians(0)
transformation= np.array([[round(math.cos(Y),4) , round(-math.sin(Y), 4), 0, x], [round(math.sin(Y),4), round(math.cos(Y), 4), 0, y],[0,0,1,z],[0,0,0,1]]) 
# translation = np.array([[x],[y],[z]])
print(transformation)
u = np.array([[x1],[y1],[z1],[1]])
u0= np.matmul(transformation,u)
print(u0)
# u1 = np.matmul(transformation,u0)
# print(u1)
transformation1= np.array([[round(math.cos(Y1),4) , round(-math.sin(Y1), 4), 0, x1], [round(math.sin(Y1),4), round(math.cos(Y1), 4), 0, y1],[0,0,1,z1],[0,0,0,1]]) 
u0=np.matmul(transformation,transformation1)

print(u0)
