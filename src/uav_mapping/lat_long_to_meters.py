import geopy.distance
import geopy.location

coords_1 = (33.636864, 72.989602)
coords_2 = (33.63686399993236, 72.98973134373007)


line1 = geopy.distance.geodesic(coords_1, coords_2).m
print (line1)

print(100 - ((line1/12)*100))