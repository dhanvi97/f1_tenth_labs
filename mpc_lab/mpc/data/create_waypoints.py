import numpy as np
from scipy.interpolate import splprep, splrep, splev
import matplotlib.pyplot as plt
from shapely import LineString, Point

f = open('waypoints.txt', 'r')

#read each line of the file
# for each line, split the line into a list of strings delimited by comma
# convert each string to a float
# convert the list of strings to a numpy array
lines = f.readlines()
waypoints = []
for line in lines:
    line = line.split(',')
    line = [float(i) for i in line]
    waypoints.append(line)

waypoints = np.array(waypoints)
f.close()

# interpolate between waypoints by 100 points using np.linspace
# save the interpolated waypoints to a csv file

# Use cubic spline to interpolate between waypoints by 100 points

# cs = splrep(waypoints[:,0], waypoints[:,1])
nominal_velocity = 0.1 # m/s
delta_waypoints = np.roll(waypoints, -1, axis=0)
d = np.linalg.norm(waypoints - delta_waypoints, axis = 1, keepdims=True)
# print(d)
delta_t = d / nominal_velocity
delta_t = np.roll(delta_t, 1, axis=0)
t = np.cumsum(delta_t)
total_time = t[-1]
# print(t)
frequency = 1000 # Hz
num_samples = int(total_time * frequency)

t_array = np.linspace(0, total_time, num_samples)

cs_x = splrep(x=t, y=waypoints[:,0])
cs_y = splrep(x=t, y=waypoints[:,1])

xs = splev(t_array, cs_x)
ys = splev(t_array, cs_y)
points = np.vstack((xs, ys)).T

contour = LineString(points)

# use shapely to interpolate along the curve at intervals of 0.03 m
interpolated_points = []
for i in np.arange(0, contour.length, 0.01):
    interpolated_points.append(contour.interpolate(i, normalized=False).coords[0])

interpolated_points = np.array(interpolated_points)
print(interpolated_points)


# dxt = splev(t_array, cs_x, der=1)
# dyt = splev(t_array, cs_y, der=1)

interpolated_diff = np.roll(interpolated_points, -100, axis=0) - np.roll(interpolated_points, 100, axis=0)
theta = np.arctan2(interpolated_diff[:,1], interpolated_diff[:,0])
plt.plot(theta)
plt.show()
# ddxt = splev(t_array, cs_x, der=2)
# ddyt = splev(t_array, cs_y, der=2)
# K = (dxt*ddyt - dyt*ddxt) / (dxt**2 + dyt**2)**(3/2)

# velocity = np.sqrt(dxt**2 + dyt**2)
# plt.plot(t_array, velocity)
# plt.show()

f = open('waypoints.csv', 'w')
for i in range(interpolated_points.shape[0]):
    f.write(str(round(interpolated_points[i, 0], 3)) + ',' + str(round(interpolated_points[i, 1], 3)) + ',' + str(round(theta[i], 3)) +  '\n')
f.close()