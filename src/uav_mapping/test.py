import numpy as np 
print("######################")
grid = np.zeros((2000, 2000))
print(grid)
print(grid.size)
print(grid)
flat_grid = grid.reshape((grid.size,)) 
print("######################")
print(flat_grid)
print(flat_grid.shape)
print("######################")
round_grid = np.round(flat_grid)
print(round_grid)
print(round_grid.shape)

# test = np.uint8(round_grid)
# print(test)
# print(test.shape)
print("######################")
# list_flat_grid = list(round_grid)
# print(list_flat_grid)
# print(type(list_flat_grid[1]))


pixel = 101
print(pixel)
robot_x= 50
robot_y=50
grid[robot_x-int(round(pixel/2)):robot_x+int(round(pixel/2)),robot_y-int(round(pixel/2)):robot_y+int(round(pixel/2))] = 1
print(grid)
print(grid.shape)




resolution =0.02 # m/cell
import math

altitude =10
fov = 40

radius = math.sqrt(((altitude/math.cos(math.radians(fov/2)))**2) - altitude**2)

print(radius)

pixel = radius/resolution

print(pixel)




